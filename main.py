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


DEFAULT_SPEED = 500


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


def undefined_dealing(color_sensor):
    sensor_color = color_sensor
    if sensor_color[0] == "Undefined" or sensor_color[1] == "Undefined":
        if sensor_color[0] == "Undefined":
            robot.move_timed(how_long=0.6, direction="back")
            robot.rotate(30, axis="diferente")
            print("sensor_color[0]")
        elif sensor_color[1] == "Undefined":
            robot.move_timed(how_long=0.6, direction="back")
            robot.rotate(-30, axis="diferente")
            print("sensor_color[1]")


last_same_color = []
color = 0
realignment_counter = 0
rect_check = False

# 22 direita
# 13


def color_realignment(robot, color_sensor_data, infrared_sensor, speed=DEFAULT_SPEED):
    global last_same_color, color, realignment_counter, rect_check
    reverse = False

    search = color_sensor_data
    if True:
        print(math.fabs(infrared_sensor[0] - infrared_sensor[1]))

        if math.fabs(infrared_sensor[0] - infrared_sensor[1]) > 6:
            robot.rotate(90)
            return None
        return None

    if robot.in_rect:
        if (search[0] != robot.rect_color) or (search[1] != robot.rect_color):
            ev3.Sound.beep()
            ev3.Sound.beep()
            ev3.Sound.beep()
            print("Saiuuuuuuu\n\n\n")
            robot.rect_color = "Undefined"
            robot.in_rect = False
            return None

    if search[0] == search[1]:
        robot.motors.left.run_forever(speed_sp=speed)
        robot.motors.right.run_forever(speed_sp=speed)
        last_same_color = search

        if search[1] not in ["White", "Undefined", "Brown"]:
            color += 1

        if color > 20:
            print("cor > que 20\n\n")
            if search[0] == "White":
                color = 0

                robot.motors.left.stop()
                robot.motors.right.stop()

                robot.move_timed(how_long=0.3, direction="back")

                ev3.Sound.speak("Robot aligned...").wait()
                robot.in_rect = True
                robot.rect_color = robot.sensor_data("ColorSensor")[0]
                rect_check = True
                ev3.Sound.speak("The rect color is " + robot.rect_color).wait()

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


robot = Robot()


client = mqtt.Client()
client.connect("169.254.232.232", 1883, 60)


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
        while True:
            search = robot.sensor_data("ColorSensor")

            color_realignment(robot, search, robot.infrared_sensors)


            if search[0] == "Undefined" and search[1] == "Undefined":
                robot.motors.left.stop()
                robot.motors.right.stop()

    except KeyboardInterrupt:
        robot.motors.right.stop()
        robot.motors.left.stop()
        client.loop_stop()
        client.disconnect()


# 23
# 18


if __name__ == '__main__':
    main()
