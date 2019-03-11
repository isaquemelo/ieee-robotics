from .duo import Duo
import ev3dev.ev3 as ev3
import math
from datetime import datetime, timedelta

DEFAULT_SPEED = 400


def map_values(n, start1, stop1, start2, stop2):
    return ((n - start1) / (stop1 - start1)) * (stop2 - start2) + start2


class Robot:
    ev3.Sound.speak("Robot started...")


    def __init__(self):
        # define sensors
        self.gyroscope_sensor = ev3.GyroSensor('in1')
        self.gyroscope_sensor.mode = 'GYRO-ANG'
        self.DEFAULT_SPEED = DEFAULT_SPEED
        self.color_sensors = Duo(ev3.ColorSensor('in2'), ev3.ColorSensor('in3'))
        self.ultrasonic_sensors = ev3.UltrasonicSensor('in4')

        # define motors
        self.motors = Duo(ev3.LargeMotor('outA'), ev3.LargeMotor('outD'), ev3.LargeMotor('outC'))
        self.motors.alternative.run_forever(speed_sp=-100)
        # self.handler = ev3.LargeMotor('outC')

        # define status
        self.in_rect = False
        self.rect_color = "Undefined"
        self.reverse_path = False
        self.dor_open = True

        # define network sensors
        self.infrared_sensors = (0, 0)
        self.ultrasonic_sensor = 255
        self.white_counter = 0

    def update(self):
        # sensors update
        if self.color_sensors.left.color == 6 and self.color_sensors.right.color == 6:
            self.white_counter += 1
        if self.white_counter > 20:
            global realignment_counter
            realignment_counter = 0
            self.white_counter = 0
        # history update
        # position update
        pass

    def sensor_data(self, sensor_name):
        # returns the value of a sensor

        if sensor_name == "InfraredSensor":
            return self.infrared_sensors.left.value(), self.infrared_sensors.right.value()

        elif sensor_name == "GyroSensor":
            return self.gyroscope_sensor.angle

        elif sensor_name == "Ultrasonic":
            return self.ultrasonic_sensors.value()/10

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

        if angle < 30 and angle > 0:
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
            self.motors.left.run_forever(speed_sp=vel)
            self.motors.right.run_forever(speed_sp=vel)
        # print("Time is over!")
        self.motors.left.stop()
        self.motors.right.stop()

    def run_action(self, direction, still_learning=True):
        if self.reverse_path:
            if not still_learning:
                if direction == "forward":
                    pass
                elif direction == "left":
                    self.rotate(90, axis="own")
                elif direction == "right":
                    self.rotate(-90, axis="own")
            else:
                self.rotate(-90, axis="own")
            return None


        else:
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

    def stop_motors(self):
        self.motors.left.stop()
        self.motors.right.stop()