import ev3dev.ev3 as ev3


# class robot
# - sensors
# - move
# - rotate
# - atual pos
# - history

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
            return dict_colors[self.color_sensors.left.color], dict_colors[self.color_sensors.right.color]

    def move(self, time, speed=DEFAULT_SPEED):
        pass

    def rotate(self, angle, speed=DEFAULT_SPEED):

        reverse = False
        if angle < 0:
            reverse = True
            angle = angle * -1

        self.gyroscope_sensor.mode = 'GYRO-RATE'
        self.gyroscope_sensor.mode = 'GYRO-ANG'

        start_angle = self.sensor_data('GyroSensor')
        print("start_angle:", start_angle)
        now_angle = start_angle

        self.motors.left.stop()
        self.motors.right.stop()

        while now_angle < angle + start_angle:
            print("now angle:", now_angle, "goal: |", angle + start_angle, "|")

            if reverse:
                self.motors.left.run_forever(speed_sp=-speed)
                self.motors.right.run_forever(speed_sp=speed)
            else:
                self.motors.left.run_forever(speed_sp=speed)
                self.motors.right.run_forever(speed_sp=-speed)

            now_angle = self.sensor_data('GyroSensor')

        self.motors.left.stop()
        self.motors.right.stop()

        self.gyroscope_sensor.mode = 'GYRO-RATE'
        self.gyroscope_sensor.mode = 'GYRO-ANG'


robot = Robot()
while True:
    valor = int(input())
    robot.rotate(valor)
    #print(robot.sensor_data("ColorSensor"))
