import ev3dev.ev3 as ev3
from assets.classes.duo import Duo
import ev3dev.ev3 as ev3
import math
from datetime import datetime, timedelta
from simple_pid import PID
import time
DEFAULT_SPEED = 400


class Robot:
    ev3.Sound.speak("Robot started...")

    def __init__(self):
        # define sensors
        self.gyroscope_sensor = ev3.GyroSensor('in3')
        self.gyroscope_sensor.mode = 'GYRO-ANG'
        self.DEFAULT_SPEED = DEFAULT_SPEED
        self.color_sensors = Duo(ev3.ColorSensor('in1'), ev3.ColorSensor('in2'))
        self.ultrasonic_sensor = ev3.UltrasonicSensor('in4')

        # define motors
        self.motors = Duo(ev3.LargeMotor('outA'), ev3.LargeMotor('outB'), ev3.LargeMotor('outC'))
        self.motors.alternative.run_forever(speed_sp=-1000)
        # self.handler = ev3.LargeMotor('outC')

        # define status
        self.in_rect = False
        self.rect_color = "Undefined"
        self.reverse_path = False
        self.dor_open = True

        # define network sensors
        self.infrared_sensors = (0, 0)
        #self.ultrasonic_sensor = 255
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
            return self.ultrasonic_sensor.value()/10

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

                    # remover dps
                    robot.motors.alternative.stop()
                    robot.motors.alternative.run_forever(speed_sp=-speed)
                    robot.move_timed(how_long=1.4, direction="back", speed=speed)
                    robot.motors.alternative.stop()
                    robot.dor_open = False

                    return


            break

        robot.motors.right.run_forever(speed_sp=speed)
        robot.motors.left.run_forever(speed_sp=speed)



robot = Robot()


ultrasonic_sensor = ev3.UltrasonicSensor('in4')

while True:
    robot.motors.left.run_forever(speed_sp=325)
    robot.motors.right.run_forever(speed_sp=325)

    if ultrasonic_sensor.value()/10 < 20:
        robot.stop_motors()
        rescue(robot)
        time.sleep(3)
