#!/usr/bin/python3

import ev3dev.ev3 as ev3
import math
from datetime import datetime, timedelta
import time
import paho.mqtt.client as mqtt
from struct import *
from simple_pid import PID


def map_values(n, start1, stop1, start2, stop2):
    return ((n - start1) / (stop1 - start1)) * (stop2 - start2) + start2


class Duo:
    def __init__(self, sensor_left, sensor_right, sensor_back=None):
        self.left = sensor_left
        self.right = sensor_right

        if sensor_back is not None:
            self.back = sensor_back

        self.values = (self.left, self.right)


DEFAULT_SPEED = 400


class Robot:
    ev3.Sound.speak("Robot started...")

    def __init__(self):
        # define sensors
        self.gyroscope_sensor = ev3.GyroSensor('in1')
        self.gyroscope_sensor.mode = 'GYRO-ANG'
        self.color_sensors = Duo(ev3.ColorSensor('in2'), ev3.ColorSensor('in3'))
        # self.ultrasonic_sensors = ev3.UltrasonicSensor('in4')

        # define motors
        self.motors = Duo(ev3.LargeMotor('outA'), ev3.LargeMotor('outB'))
        self.handler = ev3.LargeMotor('outC')

        # define status
        self.in_rect = False
        self.rect_color = "Undefined"

        # define position
        self.infrared_sensors = (0, 0)

    def update(self):
        # sensors update
        # history update
        # position update
        pass

    def sensor_data(self, sensor_name):
        # returns the value of a sensor

        if sensor_name == "InfraredSensor":
            return self.infrared_sensors.left.value(), self.infrared_sensors.right.value()

        elif sensor_name == "GyroSensor":
            return self.gyroscope_sensor.angle

        elif sensor_name == "ColorSensor":
            dict_colors = {

                0: 'Undefined',
                1: 'Black',
                2: 'Blue',
                3: 'Green',
                4: 'Yellow',
                5: 'Red',
                6: 'White',
                7: 'Brown'
            }

            # print([dict_colors[self.color_sensors.left.color], dict_colors[self.color_sensors.right.color]])
            return [dict_colors[self.color_sensors.left.color], dict_colors[self.color_sensors.right.color]]

    def rotate(self, angle, axis="own", speed=DEFAULT_SPEED):

        if angle < 30:
            speed = map_values(math.fabs(angle), 0, 90, 100, 1000)

        reverse = False
        if angle < 0:
            reverse = True
            angle = angle * -1

        self.gyroscope_sensor.mode = 'GYRO-RATE'
        self.gyroscope_sensor.mode = 'GYRO-ANG'

        start_angle = self.sensor_data('GyroSensor')
        # print("start_angle:", start_angle)
        now_angle = start_angle

        self.motors.left.stop()
        self.motors.right.stop()

        while now_angle < angle + start_angle:
            # print("now angle:", now_angle, "goal: |", angle + start_angle, "|")

            if reverse:
                if axis == "own":
                    self.motors.left.run_forever(speed_sp=-speed)
                    self.motors.right.run_forever(speed_sp=speed)
                else:
                    self.motors.right.run_forever(speed_sp=speed)

                now_angle = self.sensor_data('GyroSensor') * -1
            else:
                if axis == "own":
                    self.motors.left.run_forever(speed_sp=speed)
                    self.motors.right.run_forever(speed_sp=-speed)
                else:
                    self.motors.left.run_forever(speed_sp=speed)
                now_angle = self.sensor_data('GyroSensor')

        self.motors.left.stop()
        self.motors.right.stop()

        self.gyroscope_sensor.mode = 'GYRO-RATE'
        self.gyroscope_sensor.mode = 'GYRO-ANG'

    def move_timed(self, how_long=0.3, direction="forward", speed=DEFAULT_SPEED):
        end_time = datetime.now() + timedelta(seconds=how_long)

        vel = speed

        if direction != "forward":
            vel = -speed

        # print("Starting time!")
        while datetime.now() < end_time:
            robot.motors.left.run_forever(speed_sp=vel)
            robot.motors.right.run_forever(speed_sp=vel)
        # print("Time is over!")
        robot.motors.left.stop()
        robot.motors.right.stop()

    def run_action(self, direction, still_learning=True):
        if not still_learning:
            if direction == "forward":
                pass
            elif direction == "left":
                self.rotate(-90, axis="own")
            elif direction == "right":
                self.rotate(90, axis="own")
        else:
            self.rotate(90, axis="own")
        return None


def undefined_dealing(color_sensor):
    sensor_color = color_sensor
    if sensor_color[0] == "Undefined" or sensor_color[1] == "Undefined":
        if sensor_color[0] == "Undefined":
            robot.move_timed(how_long=0.6, direction="back")
            robot.rotate(30, axis="diferente")
        elif sensor_color[1] == "Undefined":
            robot.move_timed(how_long=0.6, direction="back")
            robot.rotate(-30, axis="diferente")


last_same_color = []
color = 0
realignment_counter = 0
rect_check = False

# 15.6 0 4.8
pid = PID(15.6, 0, 4.8, setpoint=-4)


def color_realignment(robot, color_sensor_data, infrared_sensor, move_forward=True, speed=DEFAULT_SPEED):
    global last_same_color, color, realignment_counter, rect_check
    reverse = False

    search = color_sensor_data

    if robot.in_rect:
        if (search[0] != robot.rect_color) or (search[1] != robot.rect_color):
            # print("Saiuuuuuuu\n\n\n")
            robot.rect_color = "Undefined"
            robot.in_rect = False
            return None

    if move_forward:
        if search[0] == search[1]:
            pid.output_limits = (-600, 600)
            control = pid(robot.infrared_sensors[1] - robot.infrared_sensors[0])
            n_speed = 400

            if control > 600:
                control = 600
            if control < -600:
                control = -600

            if robot.sensor_data("ColorSensor")[0] != "White" and robot.sensor_data("ColorSensor")[1] != "White":
                robot.motors.left.run_forever(speed_sp=speed)
                robot.motors.right.run_forever(speed_sp=speed)
            else:
                robot.motors.left.run_forever(speed_sp=n_speed + control)
                robot.motors.right.run_forever(speed_sp=n_speed - control)

            last_same_color = search

            if search[1] not in ["White", "Undefined", "Brown"]:
                color += 1

            if color > 15:
                if search[0] in ["White", "Undefined"] or search[0] in ["White", "Undefined"]:
                    color = 0

                    robot.motors.left.stop()
                    robot.motors.right.stop()

                    robot.move_timed(how_long=0.4, direction="back")

                    # ev3.Sound.speak("Robot aligned...").wait()
                    robot.in_rect = True
                    robot.rect_color = robot.sensor_data("ColorSensor")[0]
                    rect_check = True
                    # ev3.Sound.speak("The rect color is " + robot.rect_color).wait()
                    return "On square"

    if last_same_color[0] == "White" and last_same_color[1] == "White":
        reverse = True
    else:
        reverse = False

    if search[0] == "White" and search[1] != "White":

        robot.motors.left.stop()
        robot.motors.right.stop()

        while search[0] != search[1]:
            if reverse:
                robot.motors.left.run_forever(speed_sp=speed)
            else:
                robot.motors.right.run_forever(speed_sp=speed)

            search = robot.sensor_data("ColorSensor")
            undefined_dealing(search)

        time.sleep(0.15)
        robot.move_timed(direction="back")
        realignment_counter += 1

        if realignment_counter > 7:
            ev3.Sound.speak("Robot has exceed correction numbers...").wait()
            realignment_counter = 0
            robot.move_timed(direction="forward", how_long=0.6)

        if reverse:
            robot.motors.left.stop()
        else:
            robot.motors.right.stop()

    elif search[0] != "White" and search[1] == "White":
        robot.motors.left.stop()
        robot.motors.right.stop()

        while search[0] != search[1]:
            if reverse:
                robot.motors.right.run_forever(speed_sp=speed)
            else:
                robot.motors.left.run_forever(speed_sp=speed)

            search = robot.sensor_data("ColorSensor")
            undefined_dealing(search)

        time.sleep(0.15)
        robot.move_timed(direction="back")
        realignment_counter += 1

        if realignment_counter > 7:
            ev3.Sound.beep().wait()
            ev3.Sound.beep().wait()
            realignment_counter = 0
            robot.move_timed(direction="forward", how_long=0.6)

        if reverse:
            robot.motors.right.stop()
        else:
            robot.motors.left.stop()


def return_last_color(robot, square_color, last_choice):
    robot.rotate(180)
    while True:
        print("realignment_counter:", realignment_counter)
        search = robot.sensor_data("ColorSensor")
        result = color_realignment(robot, search, robot.infrared_sensors)

        if result == "On square" and robot.rect_color == square_color:
            print("TERMNOU IF")
            robot.in_rect = True
            break

    print("\n\nEntregue\n\n")


robot = Robot()

client = mqtt.Client()
client.connect("169.254.31.51", 1883, 60)


def on_message(client, userdata, message):
    carga = unpack("iid", message.payload)
    robot.infrared_sensors = carga[:2]
    # print("Received message:", carga[:2], time.time() - float(carga[2]))


def on_connect(client, userdata, flags, rc):
    print("The robots are connected with result code", str(rc))
    client.subscribe("topic/sensors")


client.on_connect = on_connect
client.on_message = on_message

client.loop_start()


def main():
    try:
        # learned_colors = {'Green': 'right', 'Red': 'left', 'Blue': 'forward'}
        learned_colors = {}

        being_learned = "Undefined"

        # learning_dic = {"Green": ["left", "right", "forward"]}
        learning_dic = {}
        im_learning = False
        result = None
        global realignment_counter
        while True:
            print("realignment_counter:", realignment_counter)
            search = robot.sensor_data("ColorSensor")

            result = color_realignment(robot, search, robot.infrared_sensors)
            # print(color)
            if result == "On square" or robot.in_rect:
                print("On square")
                # if search[0] != being_learned and search[0] not in ["White", "Undefined"]:
                #     learned_colors[being_learned] = learning_dic[being_learned][0]
                #     im_learning = False
                #     being_learned = "Undefined"
                #     print("Aprendi uma nova cor, segue o dicionario:", learned_colors)

                if not im_learning:
                    if robot.rect_color != "White":
                        try:
                            print("Tentando executar acao para a cor:", robot.rect_color)
                            print("Aprendidos ate agora:", learned_colors)
                            realignment_counter = 0

                            if learned_colors[robot.rect_color]:
                                robot.run_action(learned_colors[robot.rect_color], im_learning)
                                robot.move_timed(how_long=0.4)
                        except:
                            print("Acao para a cor:", robot.rect_color, "nao existe ou falhou!")
                            being_learned = robot.rect_color
                            learning_dic[being_learned] = ["right", "forward", "left"]
                            im_learning = True
                            realignment_counter = 0

                elif im_learning:
                    robot.run_action(learning_dic[being_learned][0], im_learning)
                    while True:
                        print("realignment_counter:", realignment_counter)
                        search = robot.sensor_data("ColorSensor")
                        result = color_realignment(robot, search, robot.infrared_sensors)

                        # print("On square loop secundario")
                        if robot.sensor_data("ColorSensor")[0] == robot.sensor_data("ColorSensor")[1] and \
                                robot.sensor_data("ColorSensor")[0] != being_learned and \
                                robot.sensor_data("ColorSensor")[0] not in ["White", "Undefined"] and \
                                robot.sensor_data("ColorSensor")[0] != "Black":
                            learned_colors[being_learned] = learning_dic[being_learned][0]
                            im_learning = False
                            being_learned = "Undefined"
                            learning_dic = {}
                            print("Aprendi uma nova cor, segue o dicionario:", learned_colors)
                            break

                        if search[0] == "Black" and search[1] == "Black":
                            print("DIRECAO ERRADA")
                            realignment_counter = 0
                            last_choise = learning_dic[being_learned][0]
                            del learning_dic[being_learned][0]

                            return_last_color(robot, being_learned, last_choise)

                            # inicia processo de retorno

                            """
                                if len(learning_dic[being_learned]) == 1:
                                learned_colors[being_learned] = learning_dic[being_learned][0]
                                im_learning = False
                                being_learned = "Undefined"
                                learning_dic = {}
                                cooisa = False
                                print("Aprendi uma nova cor, segue o dicionario (POR EXCLUSSAO):", learned_colors)
                            """

                            print(learning_dic)
                            break
            """
            if search[0] == "Undefined" and search[1] == "Undefined":
                robot.motors.left.stop()
                robot.motors.right.stop()
            """

    except KeyboardInterrupt:
        robot.motors.right.stop()
        robot.motors.left.stop()
        client.loop_stop()
        client.disconnect()


# 23
# 18


if __name__ == '__main__':
    main()