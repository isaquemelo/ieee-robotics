#!usr/bin/python3

import ev3dev.ev3 as ev3
import math
from datetime import datetime, timedelta
import time
import paho.mqtt.client as mqtt
from struct import *
from simple_pid import PID
from assets.classes.robot import Robot
from rescue import rescue


DEFAULT_SPEED = 400


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
realignment_counter = 0
rect_check = False

# 15.6 0 4.8
pid = PID(15.6, 0, 4.8, setpoint=-4)


def color_realignment(robot, color_sensor_data, infrared_sensor, move_forward=True, speed=DEFAULT_SPEED):
    robot.update()
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
            n_speed = 600

            if control > 400:
                control = 400
            if control < -400:
                control = -400

            # n_speed = 400
            #
            # if control > 600:
            #     control = 600
            # if control < -600:
            #     control = -600

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

                    # ev3.Sound.speak("Robot aligned...").wait()
                    robot.in_rect = True
                    # time.sleep(3) Tempo para verificar se ele está voltando de fato para dentro do quadrado com os sensores de cor.
                    if robot.sensor_data("ColorSensor")[0] not in ["White", "Undefined"]:
                        robot.rect_color = robot.sensor_data("ColorSensor")[0]
                    elif robot.sensor_data("ColorSensor")[1] not in ["White", "Undefined"]:
                        robot.rect_color = robot.sensor_data("ColorSensor")[1]

                    print("Estou num quadrado: ", robot.rect_color)
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
        robot.update()
        search = robot.sensor_data("ColorSensor")
        result = color_realignment(robot, search, robot.infrared_sensors)

        if result == "On square" and robot.rect_color == square_color:
            print("TERMNOU IF")
            robot.in_rect = True
            break

    print("\n\nEntregue\n\n")


robot = Robot()

client = mqtt.Client()
client.connect("169.254.173.223", 1883, 60)


def on_message(client, userdata, message):
    carga = unpack("iidd", message.payload)
    robot.infrared_sensors = carga[:2]
    robot.ultrasonic_sensor = carga[2]
    # print("Received message:", carga[:2], time.time() - float(carga[2]))


def on_connect(client, userdata, flags, rc):
    print("The robots are connected with result code", str(rc))
    client.subscribe("topic/sensors")


client.on_connect = on_connect
client.on_message = on_message

client.loop_start()


def main():
    try:
        learned_colors = {'Green': 'forward', 'Red': 'left', 'Blue': 'forward'}
        #learned_colors = {}

        being_learned = "Undefined"

        # learning_dic = {"Green": ["left", "right", "forward"]}
        learning_dic = {}
        im_learning = False
        result = None

        while True:
            # print("LOOP PRIMARIO")
            # print("TA APRENDENDO = {}".format(im_learning))
            robot.update()
            search = robot.sensor_data("ColorSensor")

            result = color_realignment(robot, search, robot.infrared_sensors)

            if robot.ultrasonic_sensor < 30:
                robot.stop_motors()
                rescue(robot)
                time.sleep(3)

            white_counter = 0

            if result == "On square" or robot.in_rect:
                print("On square")
                # if search[0] != being_learned and search[0] not in ["White", "Undefined"]:
                #     learned_colors[being_learned] = learning_dic[being_learned][0]
                #     im_learning = False
                #     being_learned = "Undefined"
                #     print("Aprendi uma nova cor, segue o dicionario:", learned_colors)

                if not im_learning:
                    # print("SE O ROBO DEU O BUG ESPERADO JA SEI QUE TEM QUE MEXER COM A robot.rect_colors")
                    if robot.rect_color not in ["White", "Undefined", "Black"]:
                        # print("ENTROU NO IF LOGO NAO DEU O BUG")
                        try:
                            print("Tentando executar acao para a cor:", robot.rect_color)
                            print("Aprendidos ate agora:", learned_colors)

                            if learned_colors[robot.rect_color] and robot.sensor_data("ColorSensor") not in ["Black",
                                                                                                             "Brown",
                                                                                                             "Undefined"]:
                                robot.run_action(learned_colors[robot.rect_color], im_learning)
                                robot.move_timed(how_long=0.4)
                                color = 0
                                time.sleep(0.5)

                        except:
                            # print("ENTROU NO EXCEPT")
                            print("Acao para a cor:", robot.rect_color, "nao existe ou falhou!")
                            being_learned = robot.rect_color
                            learning_dic[being_learned] = ["right", "forward", "left"]
                            im_learning = True

                elif im_learning:
                    robot.run_action(learning_dic[being_learned][0], im_learning)
                    while True:
                        # print("LOOP SECUNDARIO")
                        robot.update()
                        search = robot.sensor_data("ColorSensor")
                        result = color_realignment(robot, search, robot.infrared_sensors)

                        if robot.sensor_data("ColorSensor")[0] == robot.sensor_data("ColorSensor")[1] and \
                                robot.sensor_data("ColorSensor")[1] == "White":
                            white_counter += 1

                        print(white_counter)

                        # print("On square loop secundario")
                        if robot.sensor_data("ColorSensor")[0] == robot.sensor_data("ColorSensor")[1] and \
                                robot.sensor_data("ColorSensor")[0] not in ["White", "Undefined", "Black"]:
                            if robot.sensor_data("ColorSensor")[0] != being_learned or (
                                    robot.sensor_data("ColorSensor")[0] == being_learned and white_counter >= 5):
                                learned_colors[being_learned] = learning_dic[being_learned][0]
                                im_learning = False
                                being_learned = "Undefined"
                                learning_dic = {}
                                print("Aprendi uma nova cor, segue o dicionario:", learned_colors)
                                break

                        if search[0] == "Black" and search[1] == "Black":
                            print("DIRECAO ERRADA")
                            global realignment_counter
                            robot.motors.left.stop()
                            robot.motors.right.stop()
                            time.sleep(0.3)
                            realignment_counter = 0

                            last_choise = learning_dic[being_learned][0]
                            del learning_dic[being_learned][0]

                            return_last_color(robot, being_learned, last_choise)

                            print("learned_dic =", learning_dic)
                            break

                        """
                        if len(learning_dic[being_learned]) == 1:
                            learned_colors[being_learned] = learning_dic[being_learned][0]
                            im_learning = False
                            being_learned = "Undefined"
                            learning_dic = {}
                            cooisa = False
                            print("Aprendi uma nova cor, segue o dicionario (POR EXCLUSSAO):", learned_colors)

                            print("learned_dic = {}".format(learning_dic))
                            break
                        """

                    white_counter = 0

            if search[0] == "Undefined" and search[1] == "Undefined":
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