import ev3dev.ev3 as ev3
from assets.classes.robot import Robot


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
        robot.motors.alternative.stop()
        robot.motors.alternative.run_forever(speed_sp=-1000)
        robot.move_timed(how_long=1.4, direction="back", speed=speed)
        robot.motors.alternative.stop()
        robot.dor_open = False
        robot.has_doll = False


def bounding_box(robot, speed=DEFAULT_SPEED):
    while True:
        # print("Bouding box loop..")
        search = robot.sensor_data("ColorSensor")
        if search[0] == "Black" or search[1] == "Black":
            robot.move_timed(how_long=1.3, direction="forward", speed=speed)
            drop_doll(robot)
            robot.rotate(180)
            robot.reverse_path = True
            robot.move_timed(how_long=5.3, direction="forward", speed=speed)
            break

    return

