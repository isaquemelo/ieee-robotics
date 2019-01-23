import ev3dev.ev3 as ev3
import math
from datetime import datetime, timedelta
import time


# class robot
# - sensors
# - move
# - rotate
# - atual pos
# - history

def map_values(n, start1, stop1, start2, stop2):
    return ((n - start1) / (stop1 - start1)) * (stop2 - start2) + start2


class Duo:
    def __init__(self, sensor_left, sensor_right):
        self.left = sensor_left
        self.right = sensor_right
        self.values = (self.left, self.right)


class Robot:
    DEFAULT_SPEED = 1000
    print("robot")

    def __init__(self):
        # define sensors
        self.gyroscope_sensor = ev3.GyroSensor('in1')
        self.gyroscope_sensor.mode = 'GYRO-ANG'
        self.color_sensors = Duo(ev3.ColorSensor('in2'), ev3.ColorSensor('in3'))
        # self.infrared_sensors = Duo(ev3.InfraredSensor('in3'), ev3.InfraredSensor('in4'))

        # self.ultrasonic_sensors = Duo(ev3.UltrasonicSensor('in'), ev3.UltrasonicSensor('in'))

        # define motors
        self.motors = Duo(ev3.LargeMotor('outA'), ev3.LargeMotor('outB'))
        # define history
        # define position

        pass

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
            print([dict_colors[self.color_sensors.left.color], dict_colors[self.color_sensors.right.color]])
            return [dict_colors[self.color_sensors.left.color], dict_colors[self.color_sensors.right.color]]

    def move(self, time, speed=DEFAULT_SPEED):

        pass

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

    def move_back_timed(self, how_long=0.3):
        end_time = datetime.now() + timedelta(seconds=how_long)
        print("Starting time!")
        while datetime.now() < end_time:
            robot.motors.left.run_forever(speed_sp=-400)
            robot.motors.right.run_forever(speed_sp=-400)
        print("Time is over!")
        robot.motors.left.stop()
        robot.motors.right.stop()


robot = Robot()
# while True:
#     count = 0
#     start_time = 0
#     while True:
#
#         search = robot.sensor_data("ColorSensor")
#
#         if search[1] == "White" and search[0] != "White" and not (search[0] == "Undefined" or search[0] == "Brown"):
#             while search[1] != search[0]:
#                 robot.rotate(-3, axis="rato")
#                 search = robot.sensor_data("ColorSensor")
#
#         elif search[0] == "White" and search[1] != "White" and not (search[1] == "Undefined" or search[1] == "Brown"):
#             while search[0] != search[1]:
#                 robot.rotate(3, axis="rato")
#                 search = robot.sensor_data("ColorSensor")
#
#         elif search[0] == search[1] and search[0] != "White":

#         else:
#             if search == ["White", "White"]:
#                 robot.motors.left.run_forever(speed_sp=1000)
#                 robot.motors.right.run_forever(speed_sp=1000)
#
#     robot.motors.left.stop()
#     robot.motors.right.stop()
#     break

while True:

    search = robot.sensor_data("ColorSensor")

    if search[0] == search[1]:
        robot.motors.left.run_forever(speed_sp=800)
        robot.motors.right.run_forever(speed_sp=800)
        last_same_color = search

    if last_same_color[0] == "White" and var[1] == "White":

        if search[0] == "White" and search[1] != "White":
            print("A")
            robot.motors.left.stop()
            robot.motors.right.stop()
            counter = 0

            while search[0] != search[1] or counter < 5:
                counter += 1
                robot.motors.left.run_forever(speed_sp=800)
                search = robot.sensor_data("ColorSensor")

            robot.move_back_timed()
            robot.motors.left.stop()

        elif search[0] != "White" and search[1] == "White":
            print("B")
            robot.motors.left.stop()
            robot.motors.right.stop()

            while search[0] != search[1]:
                robot.motors.right.run_forever(speed_sp=800)
                search = robot.sensor_data("ColorSensor")

            robot.move_back_timed()
            robot.motors.right.stop()

    else:

        if search[0] == "White" and search[1] != "White":
            print("A")
            robot.motors.left.stop()
            robot.motors.right.stop()
            counter = 0
            while search[0] != search[1] or counter < 5:
                counter += 1
                robot.motors.right.run_forever(speed_sp=800)
                search = robot.sensor_data("ColorSensor")

            robot.move_back_timed()
            robot.motors.left.stop()

        elif search[0] != "White" and search[1] == "White":
            print("B")
            robot.motors.left.stop()
            robot.motors.right.stop()
            while search[0] != search[1]:
                robot.motors.left.run_forever(speed_sp=800)
                search = robot.sensor_data("ColorSensor")

            robot.move_back_timed()
            robot.motors.right.stop()

    if search[0] == "Undefined" and search[1] == "Undefined":
        robot.motors.left.stop()
        robot.motors.right.stop()
