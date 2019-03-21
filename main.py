#!usr/bin/python3

import ev3dev.ev3 as ev3
import math
from datetime import datetime, timedelta
import time
import paho.mqtt.client as mqtt
from struct import *
from simple_pid import PID
from assets.classes.robot import Robot
from rescue import rescue, bounding_box

DEFAULT_SPEED = 350


def undefined_dealing(color_sensor):
    sensor_color = color_sensor
    if sensor_color[0] == "Undefined" or sensor_color[1] == "Undefined":
        if sensor_color[0] == "Undefined":
            robot.move_timed(how_long=0.6, direction="back")
            robot.rotate(30, axis="diferente")
        elif sensor_color[1] == "Undefined":
            robot.move_timed(how_long=0.6, direction="back")
            robot.rotate(-30, axis="diferente")


last_same_color = [None, ""]
color = 0
rect_check = False

# 15.6 0 4.8
pid = PID(15.6, 0, 4.8, setpoint=-4)


def color_realignment(robot, color_sensor_data, infrared_sensor, move_forward=True, speed=DEFAULT_SPEED):
    robot.update()
    global last_same_color, color, rect_check
    reverse = False

    search = color_sensor_data
    pid.output_limits = (-600, 600)
    control = pid(robot.infrared_sensors[1] - robot.infrared_sensors[0])

    if robot.in_rect:
        if (search[0] != robot.rect_color) or (search[1] != robot.rect_color):
            robot.rect_color = "Undefined"
            robot.in_rect = False
            return None

    if robot.reverse_path is None or robot.has_doll:
        n_speed = 600
        if control > 400:
            control = 400
        if control < -400:
            control = -400
    else:
        n_speed = 280

        if control > 60:
            control = 60
        if control < -60:
            control = -60

    if search[0] == search[1]:
        print("[0] == [1]")
        if robot.sensor_data("ColorSensor")[0] != "White" and robot.sensor_data("ColorSensor")[1] != "White":
            robot.motors.left.run_forever(speed_sp=speed)
            robot.motors.right.run_forever(speed_sp=speed)
        else:
            robot.motors.left.run_forever(speed_sp=n_speed + control)
            robot.motors.right.run_forever(speed_sp=n_speed - control)

        last_same_color = search

        if search[1] not in ["White", "Undefined", "Brown", "Black"]:
            color += 1

        if color > 13:
            if search[0] in ["White", "Undefined"] or search[1] in ["White", "Undefined"]:
                color = 0

                robot.motors.left.stop()
                robot.motors.right.stop()

                robot.move_timed(how_long=0.4, direction="back")

                robot.in_rect = True
                if robot.sensor_data("ColorSensor")[0] not in ["White", "Undefined"]:
                    robot.rect_color = robot.sensor_data("ColorSensor")[0]
                elif robot.sensor_data("ColorSensor")[1] not in ["White", "Undefined"]:
                    robot.rect_color = robot.sensor_data("ColorSensor")[1]

                print("I'm on a square", robot.rect_color)
                rect_check = True
                return "On square"

    elif search[0] == "Undefined" and search[1] == "White" or  search[1] == "White" and search[0] == "Undefined":
        print("Undefine Dealing")
        undefined_dealing(search)

    elif search[0] == "White" and search[1] != "White":
        print("[0] == White and [1] != White")
        if last_same_color[0] == "White" and last_same_color[1] == "White":
            reverse = True
        else:
            reverse = False

        robot.motors.left.stop()
        robot.motors.right.stop()

        while search[0] != search[1]:
            if reverse:
                robot.motors.left.run_forever(speed_sp=speed)
            else:
                robot.motors.right.run_forever(speed_sp=speed)

            search = robot.sensor_data("ColorSensor")

        time.sleep(0.15)
        robot.move_timed(direction="back")
        robot.realigment_counter += 1

        if robot.realigment_counter > 7:
            ev3.Sound.speak("Robot has exceed correction numbers...").wait()
            robot.realigment_counter = 0
            robot.move_timed(direction="forward", how_long=0.6)

        if reverse:
            robot.motors.left.stop()
        else:
            robot.motors.right.stop()

    elif search[0] != "White" and search[1] == "White":
        print("[0] != White and [1] == White")

        if last_same_color[0] == "White" and last_same_color[1] == "White":
            reverse = True
        else:
            reverse = False

        robot.motors.left.stop()
        robot.motors.right.stop()

        while search[0] != search[1]:

            if reverse:
                robot.motors.right.run_forever(speed_sp=speed)
            else:
                robot.motors.left.run_forever(speed_sp=speed)

            search = robot.sensor_data("ColorSensor")

        time.sleep(0.15)
        robot.move_timed(direction="back")
        robot.realigment_counter += 1

        if robot.realigment_counter > 7:
            ev3.Sound.beep().wait()
            ev3.Sound.beep().wait()
            robot.realigment_counter = 0
            robot.move_timed(direction="forward", how_long=0.6)

        if reverse:
            robot.motors.right.stop()
        else:
            robot.motors.left.stop()
    else:
        print("else")
        if robot.sensor_data("ColorSensor")[0] != "White" and robot.sensor_data("ColorSensor")[1] != "White":
            robot.motors.left.run_forever(speed_sp=speed)
            robot.motors.right.run_forever(speed_sp=speed)
        else:
            robot.motors.left.run_forever(speed_sp=n_speed + control)
            robot.motors.right.run_forever(speed_sp=n_speed - control)


def return_last_color(robot, square_color, last_choice):
    robot.rotate(180)
    while True:
        robot.update()
        search = robot.sensor_data("ColorSensor")
        result = color_realignment(robot, search, robot.infrared_sensors)

        if result == "On square" and robot.rect_color == square_color:
            robot.in_rect = True
            break

    print("\n\nEntregue\n\n")


robot = Robot()

client = mqtt.Client()
client.connect("169.254.87.119", 1883, 60)


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
        #learned_colors = {'Green': 'right', 'Red': 'forward', 'Blue': 'left'}
        learned_colors = {}
        being_learned = "Undefined"
        learning_dic = {}
        im_learning = False
        result = None
        undefined_counter = 0

        while True:

            robot.update()

            if robot.reverse_path == False:
                robot.done_learning = True
                print("Starting bounding box..")
                bounding_box(robot)
                continue

            search = robot.sensor_data("ColorSensor")
            result = color_realignment(robot, search, robot.infrared_sensors)

            if robot.sensor_data("Ultrasonic") < 20 and not robot.has_doll:
                robot.stop_motors()
                rescue(robot)
                time.sleep(1)

            white_counter = 0
            #print(robot.historic)

            if result == "On square" or robot.in_rect:
                print("On square")

                if not im_learning:
                    if robot.rect_color not in ["White", "Undefined", "Black"]:
                        try:
                            print("Tentando executar acao para a cor:", robot.rect_color)
                            print("Aprendidos ate agora:", learned_colors)

                            if learned_colors[robot.rect_color] and robot.sensor_data("ColorSensor") not in ["Black",
                                                                                                             "Brown",
                                                                                                             "Undefined"]:
                                print("Executando acao:", learned_colors[robot.rect_color])
                                robot.run_action(learned_colors[robot.rect_color], im_learning)

                                # adds action to historic
                                if robot.reverse_path is None:
                                    robot.historic.append(robot.rect_color)
                                elif robot.reverse_path and robot.nao_pode:
                                    print("Reverse path is True", len(robot.historic))
                                    print("AAAAAAAAAA")
                                    if len(robot.historic) == 1:
                                        pass
                                    else:
                                        print("Removendo item")
                                        last_item = robot.historic[-1]
                                        robot.historic.pop()
                                        if len(robot.historic) == 1:
                                            print("Ultimo item")
                                            robot.rotate(90)
                                            robot.reverse_path = None
                                            robot.historic.append(last_item)

                                robot.move_timed(how_long=0.4)
                                color = 0

                                time.sleep(0.5)

                        except:
                            print("Acao para a cor:", robot.rect_color, "nao existe ou falhou!")
                            being_learned = robot.rect_color
                            learning_dic[being_learned] = ["right", "forward", "left"]
                            im_learning = True

                elif im_learning:
                    robot.run_action(learning_dic[being_learned][0], im_learning)
                    while True:
                        if robot.sensor_data("Ultrasonic") < 20 and not robot.has_doll:
                            robot.stop_motors()
                            rescue(robot)
                            time.sleep(1)

                        robot.update()
                        search = robot.sensor_data("ColorSensor")
                        result = color_realignment(robot, search, robot.infrared_sensors)

                        if robot.sensor_data("ColorSensor")[0] == robot.sensor_data("ColorSensor")[1] and \
                                robot.sensor_data("ColorSensor")[1] == "White":
                            white_counter += 1

                        color_sensor = robot.sensor_data("ColorSensor")

                        if color_sensor[0] == color_sensor[1] and \
                                color_sensor[0] not in ["White", "Undefined", "Black", "Brown"]:
                            if color_sensor[0] != being_learned or (
                                    color_sensor[0] == being_learned and white_counter >= 5):

                                learned_colors[being_learned] = learning_dic[being_learned][0]

                                # adds action to historic
                                if robot.reverse_path == None and not robot.nao_pode:
                                    print("BBBBBBBBB")
                                    robot.historic.append(being_learned)

                                im_learning = False
                                being_learned = "Undefined"
                                learning_dic = {}

                                print("Aprendi uma nova cor, segue o dicionario:", learned_colors)

                                break

                        if robot.sensor_data("ColorSensor")[0] == "Black" and robot.sensor_data("ColorSensor")[1] == "Black":
                            print("Wrong path")
                            robot.motors.left.stop()
                            robot.motors.right.stop()
                            time.sleep(0.2)
                            robot.realigment_counter = 0

                            last_choise = learning_dic[being_learned][0]
                            del learning_dic[being_learned][0]

                            return_last_color(robot, being_learned, last_choise)

                            print("learned_dic =", learning_dic)
                            break

                    white_counter = 0


            if search[0] == "Undefined" and search[1] == "Undefined":
                undefined_counter += 1

                if undefined_counter > 30:
                    undefined_counter = 0
                    print("Both sensors are undefined!")
                    robot.move_timed(how_long=0.5, direction="back")
                    robot.motors.left.stop()
                    robot.motors.right.stop()

    except KeyboardInterrupt:
        robot.motors.right.stop()
        robot.motors.left.stop()
        robot.motors.alternative.stop()
        client.loop_stop()
        client.disconnect()


if __name__ == '__main__':
    main()
