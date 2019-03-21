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

        if "Undefined" in search:
            robot.stop_motors()
            robot.move_timed(how_long=0.1, direction="forward", speed=speed)
            robot.stop_motors()
            search = robot.sensor_data("ColorSensor")
            if search[0] == "Undefined" and search[1] == "Undefined":
                robot.motors.alternative.run_forever(speed_sp=1000)
                robot.stop_motors()
                robot.move_timed(how_long=1.6, direction="back", speed=speed)
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

                        robot.motors.alternative.run_forever(speed_sp=1000)
                        return

            # camada de protessao caso o robo tente entrar com uma das rodas fora da plataforma (o robo vai tentar resgatar o boneco)
            if search[0] != search[1]:
                # tentando pegar o doll antes de chegar a ponto de cair
                robot.move_timed(how_long=0.6, direction="forward", speed=speed)
                robot.stop_motors()
                robot.motors.alternative.run_forever(speed_sp=1000)
                robot.rotate(-9, speed=1000)
                robot.rotate(9, speed=1000)
                robot.stop_motors()
                robot.move_timed(how_long=0.7, direction="back", speed=speed)

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
        robot.dor_open = False
        robot.has_doll = False


def bounding_box(robot, speed=DEFAULT_SPEED):
    kp = 20
    ki = 0.2
    kd = 60.1
    black_counter = 0
    #robot.motors.alternative.run_forever(speed_sp=1000)
    can_break = False
    #contador_para_re = 40
    # limits
    pid = PID(kp, ki, kd, setpoint=76.2)

    # 83.3 indo
    # 75 voltando

    pid.output_limits = (-400, 400)
    while True:
        print("Bouding box loop..")
        search = robot.sensor_data("ColorSensor")
        if search[0] == "Black" and search[1] == "Black":
            black_counter += 1
        if black_counter >= 70:
            drop_doll(robot)
            # move back with pid
            robot.move_timed(how_long=1.1, direction="back", speed=1000)
            robot.rotate(180)
            pid = PID(kp, ki, kd, setpoint=81.2)
            black_counter = 0
            can_break = True

        #control = pid(robot.sensor_data("Ultrasonic"))
        ultrasonico = robot.sensor_data("Ultrasonic")

        n_speed = 350
        # print("ULTRASONICO: ", ultrasonico)
        if 60 <= ultrasonico <= 100:
            control = pid(robot.sensor_data("Ultrasonic"))
            if control > 500:
                control = 500
            if control < -500:
                control = -500
            robot.motors.left.run_forever(speed_sp=n_speed + control)
            robot.motors.right.run_forever(speed_sp=n_speed - control)

        else:
            robot.motors.left.run_forever(speed_sp=n_speed)
            robot.motors.right.run_forever(speed_sp=n_speed)

        if can_break and (search[0] not in ["White", "Black", "Undefined"] or search[1] not in ["White", "Black", "Undefined"]):
            robot.reverse_path = True
            robot.move_timed(how_long=0.8, speed=500)
            break

    return
