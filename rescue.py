import ev3dev.ev3 as ev3
import math
from datetime import datetime, timedelta
import time
import paho.mqtt.client as mqtt
from struct import *
from assets.classes.robot import Robot


#
# client = mqtt.Client()
# client.connect("169.254.188.100", 1883, 60)
#
#
# def on_message(client, userdata, message):
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

# client.loop_start()
DEFAULT_SPEED = 400


def rescue(robot, speed=DEFAULT_SPEED):
    robot.motors.left.stop()
    robot.motors.right.stop()
    robot.rotate(-90, speed=300)

    while True:
        search = robot.sensor_data("ColorSensor")
        if search[0] == "Undefined" and search[1] == "Undefined":
            robot.motors.alternative.run_forever(speed_sp=800)
            robot.stop_motors()
            robot.move_timed(how_long=0.3, direction="back", speed=speed)
            robot.rotate(180, speed=1000)

            while True:
                search = robot.sensor_data("ColorSensor")
                robot.motors.right.run_forever(speed_sp=speed)
                robot.motors.left.run_forever(speed_sp=speed)
                if search[0] == "Undefined" and search[1] == "Undefined":
                    robot.stop_motors()
                    robot.move_timed(how_long=0.3, direction="back", speed=speed)
                    robot.rotate(-90, speed=300)

                    # remover dps
                    robot.motors.alternative.stop()
                    robot.motors.alternative.run_forever(speed_sp=-speed)
                    robot.move_timed(how_long=1.2, direction="back", speed=speed)
                    robot.motors.alternative.stop()

                    return


            break

        robot.motors.right.run_forever(speed_sp=speed)
        robot.motors.left.run_forever(speed_sp=speed)


robot = Robot()


def main():
    try:
        while True:
            test = input("O que que? ")
            if test == "vai":
                rescue(robot)
            else:
                pass

    except KeyboardInterrupt:
        #client.loop_stop()
        #client.disconnect()
        robot.stop_motors()
        robot.motors.alternative.stop()


if __name__ == '__main__':
    main()

