import ev3dev.ev3 as ev3
from assets.classes.robot import Robot
from simple_pid import PID

DEFAULT_SPEED = 400


def rescue(robot, speed=DEFAULT_SPEED):
    robot.motors.left.stop()
    robot.motors.right.stop()
    robot.rotate(-90, speed=300)

    # if not robot.dor_open

    robot.motors.alternative.run_timed(time_sp=1000, speed_sp=-1000)
    robot.dor_open = True

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
                if search[0] == "Undefined" or search[1] == "Undefined":
                    robot.stop_motors()
                    robot.move_timed(how_long=0.3, direction="back", speed=speed)
                    if not robot.done_learning:
                        robot.rotate(-90, speed=300)
                    robot.has_doll = True

                    if robot.done_learning:
                        robot.rotate(90, speed=500)
                        robot.reverse_path = None

                    return

        robot.motors.right.run_forever(speed_sp=speed)
        robot.motors.left.run_forever(speed_sp=speed)


def drop_doll(robot, speed=DEFAULT_SPEED):
    if robot.has_doll:
        ev3.Sound.beep()
        ev3.Sound.beep()
        robot.motors.alternative.stop()
        robot.motors.alternative.run_timed(time_sp=2000, speed_sp=-1000)
        #robot.move_timed(how_long=1.4, direction="back", speed=speed)
        #robot.motors.alternative.stop()
        robot.dor_open = False
        robot.has_doll = False

def bounding_box(robot, speed=DEFAULT_SPEED):
    black_counter = 0
    robot.motors.alternative.run_timed(time_sp=1000, speed_sp=1000)
    p_reverse = False
    # limits
    pid = PID(20, 0.2, 60.1, setpoint=83.3)

    # 83.3 indo
    # 75 voltando

    pid.output_limits = (-400, 400)
    has_seen_black = False
    while True:
        #print("Bouding box loop..")
        search = robot.sensor_data("ColorSensor")
        if search[0] == "Black" and search[1] == "Black":
            black_counter += 1
            if black_counter >= 60:
                drop_doll(robot)
                # move back with pid
                p_reverse = True
                black_counter = 0

        if p_reverse == True and "White" in search:
            p_reverse = False
            robot.rotate(180)
            pid = PID(20, 0.2, 60.1, setpoint=75)
            print(75)

        control = pid(robot.sensor_data("Ultrasonic"))
        ultrasonico = robot.sensor_data("Ultrasonic")

        # if search[0] == "Black" and search[1] == "Black" and not has_seen_black:
        #
        #     robot.move_timed(how_long=3.3, direction="forward", speed=speed)
        #     drop_doll(robot)
        #     robot.move_timed(how_long=1.6, direction="back", speed=speed)
        #     robot.rotate(180)
        #     robot.reverse_path = True
        #     pid = PID(20, 0.2, 60.1, setpoint=75)
        #
        #     ev3.Sound.beep()
        #     has_seen_black = True
        #     #return


        n_speed = 350
        #print("ULTRASONICO: ", ultrasonico)
        if ultrasonico >= 60 and ultrasonico <= 100:
            if control > 500:
                control = 500
            if control < -500:
                control = -500
            if p_reverse == False:
                robot.motors.left.run_forever(speed_sp=n_speed + control)
                robot.motors.right.run_forever(speed_sp=n_speed - control)
            else:
                robot.motors.left.run_forever(speed_sp=(n_speed + control) * (-999/(n_speed + control)))
                robot.motors.right.run_forever(speed_sp=(n_speed - control) * (-999/(n_speed - control)))
        else:
            robot.motors.left.run_forever(speed_sp=n_speed)
            robot.motors.right.run_forever(speed_sp=n_speed)



