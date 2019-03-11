import ev3dev.ev3 as ev3
import math
from datetime import datetime, timedelta
import time
import paho.mqtt.client as mqtt
from struct import *
from assets.classes.robot import Robot



# client = mqtt.Client()
# client.connect("169.254.51.126", 1883, 60)
#
# carga = []
#
# def on_message(client, userdata, message):
#     global carga
#     carga = unpack("iidd", message.payload)
#     #print("Received message:", carga[:3])
#
#
# def on_connect(client, userdata, flags, rc):
#     print("The robots are connected with result code", str(rc))
#     client.subscribe("topic/sensors")
#
#
# client.on_connect = on_connect
# client.on_message = on_message
#
#
# client.loop_start()
DEFAULT_SPEED = 400


def rescue(robot, speed=DEFAULT_SPEED):
    robot.motors.left.stop()
    robot.motors.right.stop()
    robot.rotate(-90, speed=300)

    while True:
        search = robot.sensor_data("ColorSensor")

        if robot.dor_open == False:
            robot.motors.alternative.run_timed(time_sp=1000, speed_sp=-1000)
            robot.dor_open = True

        if search[0] == "Undefined" and search[1] == "Undefined":
            robot.motors.alternative.run_forever(speed_sp=800)
            robot.stop_motors()
            robot.move_timed(how_long=0.3, direction="back", speed=speed)
            robot.rotate(180, speed=1000)

            while True:
                search = robot.sensor_data("ColorSensor")
                robot.motors.right.run_forever(speed_sp=speed)
                robot.motors.left.run_forever(speed_sp=speed)
                if search[0] == "Undefined" or search[1] == "Undefined":
                    robot.stop_motors()
                    robot.move_timed(how_long=0.3, direction="back", speed=speed)
                    robot.rotate(-90, speed=300)
                    robot.has_doll = True

                    return


            break

        robot.motors.right.run_forever(speed_sp=speed)
        robot.motors.left.run_forever(speed_sp=speed)


def drop_doll(robot, speed=DEFAULT_SPEED):
    # remover dps
    robot.motors.alternative.stop()
    robot.motors.alternative.run_forever(speed_sp=-speed)
    robot.move_timed(how_long=1.4, direction="back", speed=speed)
    robot.motors.alternative.stop()
    robot.dor_open = False
    robot.has_doll = False


def bounding_box(robot, speed=DEFAULT_SPEED):
    while True:
        print("BOUNDING BOX")
        search = robot.sensor_data("ColorSensor")
        if search[0] == "Black" and search[1] == "Black":
            robot.move_timed(how_long=1.3, direction="forward", speed=speed)
            drop_doll(robot)
            robot.rotate(180)
            robot.reverse_path = True
            robot.move_timed(how_long=5.3, direction="forward", speed=speed)
            break

    return

