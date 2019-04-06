import ev3dev.ev3 as ev3
from assets.classes.robot import Robot
from simple_pid import PID
from datetime import datetime, timedelta
import time

DEFAULT_SPEED = 400


def deal_ret(robot):
    if robot.reverse_path is False:
        robot.rotate(90, speed=300)
    elif robot.reverse_path in [True, None]:
        robot.rotate(-90, speed=300)
    robot.has_doll = True
    robot.reverse_path = False
    return

def rescue(robot, speed=DEFAULT_SPEED):
    robot.stop_motors()
    robot.rotate(-90, speed=300)
    res_dang = True
    # if not robot.dor_open

    robot.motors.alternative.run_timed(time_sp=1000, speed_sp=-1000)
    robot.dor_open = True

    end_time = datetime.now() + timedelta(seconds=1)
    while True:
        search = robot.sensor_data("ColorSensor")

        robot.motors.right.run_forever(speed_sp=speed)
        robot.motors.left.run_forever(speed_sp=speed)

        if datetime.now() >= end_time:  # se passou 1 segundo ou mais para encontrar o "Undefined"
            res_dang = False

        if "Undefined" in search:
            robot.stop_motors()
            search = robot.sensor_data("ColorSensor")
            robot.stop_motors()

            if not res_dang:
                ev3.Sound.beep()
                print("RESGATE SEGURO")
                robot.motors.alternative.run_forever(speed_sp=1000)
                robot.stop_motors()
                while "White" not in search:
                    search = robot.sensor_data("ColorSensor")
                    robot.motors.right.run_forever(speed_sp=-speed)
                    robot.motors.left.run_forever(speed_sp=-speed)
                robot.stop_motors()
                #time.sleep(0.5)
                while search[0] == "Undefined":
                    search = robot.sensor_data("ColorSensor")
                    robot.motors.left.run_forever(speed_sp=-speed)
                robot.stop_motors()
                while search[1] == "Undefined":
                    search = robot.sensor_data("ColorSensor")
                    robot.motors.right.run_forever(speed_sp=-speed)
                robot.stop_motors()
                robot.move_timed(how_long=1.5, direction="back", speed=speed)
                robot.rotate(180, speed=1000)

                while True:
                    search = robot.sensor_data("ColorSensor")
                    if "Undefined" in search:
                        robot.stop_motors()
                        robot.move_timed(how_long=0.3, direction="back", speed=speed)
                        # if not robot.done_learning:
                        #     robot.rotate(-90, speed=300)
                        # if robot.reverse_path is False:
                        #     robot.rotate(-90, speed=300)
                        #     robot.reverse_path = True
                        # # robot.has_doll = True
                        #
                        # # if robot.done_learning:
                        # #     robot.rotate(90, speed=500)
                        # #     robot.reverse_path = None
                        # else:
                        #     robot.rotate(90, speed=500)
                        #     robot.reverse_path = False
                        # robot.motors.alternative.run_forever(speed_sp=1000)
                        # robot.has_doll = True
                        deal_ret(robot)
                        return
                    else:
                        robot.motors.right.run_forever(speed_sp=speed)
                        robot.motors.left.run_forever(speed_sp=speed)

            elif res_dang:
                print("RESGATE ARISCADO")
                search = robot.sensor_data("ColorSensor")
                # camada de protessao caso o robo tente entrar com uma das rodas fora da plataforma (o robo vai tentar resgatar o boneco)
                if search[0] == "Undefined" and search[1] != "Undefined":
                    ev3.Sound.beep()
                    ev3.Sound.beep()
                    ev3.Sound.beep()
                    # tentando pegar o doll antes de chegar a ponto de cair
                    robot.move_timed(how_long=0.7, direction="forward", speed=speed)
                    robot.stop_motors()
                    #time.sleep(3)
                    robot.motors.alternative.run_forever(speed_sp=1000)
                    robot.move_timed(how_long=0.1, direction="forward", speed=speed)
                    robot.rotate(9)
                    #time.sleep(3)
                    robot.rotate(-9)
                    #time.sleep(3)
                    #time.sleep(2)
                    #robot.rotate(9, speed=1000)
                    #robot.move_timed(how_long=0.2, direction="back", speed=speed)
                    search = robot.sensor_data("ColorSensor")
                    while "Undefined" in search:
                        search = robot.sensor_data("ColorSensor")
                        robot.motors.right.run_forever(speed_sp=-speed)
                        robot.motors.left.run_forever(speed_sp=-speed)
                    robot.stop_motors()
                    #time.sleep(2)
                    #robot.rotate(9, speed=1000)
                    #robot.move_timed(how_long=0.7, direction="back", speed=speed)
                    robot.rotate(180, speed=1000)
                    while True:
                        search = robot.sensor_data("ColorSensor")
                        if "Undefined" in search:
                            robot.stop_motors()
                            robot.move_timed(how_long=0.3, direction="back", speed=speed)
                            # if not robot.done_learning:
                            #     robot.rotate(-90, speed=300)
                            # robot.has_doll = True
                            #
                            # if robot.done_learning:
                            #     robot.rotate(90, speed=500)
                            #     robot.reverse_path = None
                            # if robot.reverse_path is False:
                            #     robot.rotate(90, speed=300)
                            #     robot.reverse_path = True
                            #
                            # else:
                            #     robot.rotate(-90, speed=500)
                            #     robot.reverse_path = False

                            # robot.has_doll = True
                            deal_ret(robot)
                            return

                        else:
                            robot.motors.right.run_forever(speed_sp=speed)
                            robot.motors.left.run_forever(speed_sp=speed)


                elif search[0] != "Undefined" and search[1] == "Undefined":
                    ev3.Sound.beep()
                    ev3.Sound.beep()
                    ev3.Sound.beep()
                    # tentando pegar o doll antes de chegar a ponto de cair
                    robot.move_timed(how_long=0.7, direction="forward", speed=speed)
                    robot.stop_motors()
                    #time.sleep(3)
                    robot.motors.alternative.run_forever(speed_sp=1000)
                    robot.move_timed(how_long=0.1, direction="forward", speed=speed)
                    robot.rotate(-9)
                    robot.rotate(9)
                    # time.sleep(2)
                    # robot.rotate(9, speed=1000)
                    # time.sleep(2)
                    # robot.rotate(9, speed=1000)
                    robot.stop_motors()
                    # robot.move_timed(how_long=0.7, direction="back", speed=speed)
                    while "Undefined" in search:
                        search = robot.sensor_data("ColorSensor")
                        robot.motors.right.run_forever(speed_sp=-speed)
                        robot.motors.left.run_forever(speed_sp=-speed)
                    robot.stop_motors()
                    robot.rotate(180, speed=1000)
                    while True:
                        search = robot.sensor_data("ColorSensor")
                        if "Undefined" in search:
                            robot.stop_motors()
                            robot.move_timed(how_long=0.3, direction="back", speed=speed)
                            # if not robot.done_learning:
                            #     robot.rotate(-90, speed=300)
                            # robot.has_doll = True
                            #
                            # if robot.done_learning:
                            #     robot.rotate(90, speed=500)
                            #     robot.reverse_path = None

                            # if robot.reverse_path is False:
                            #     robot.rotate(90, speed=300)
                            #     robot.reverse_path = True
                            #
                            # else:
                            #     robot.rotate(-90, speed=500)
                            #     robot.reverse_path = False

                            # robot.has_doll = True
                            deal_ret(robot)
                            return

                        else:
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
    robot.voltou = False
    # recarrega todos as cores pelas quais vai passar voltando;
    if robot.primeiro_bounding_box is True:
        robot.primeiro_bounding_box = False
        # print("PRIMEIRA CARGA")
        keys = robot.learned_colors.keys()
        for k in keys:
            robot.learned_colors[k].append(2)
            print(robot.learned_colors)
    else:
        # print("CARGA 1 + N")
        for k in sorted(robot.learned_colors.keys()):
            robot.learned_colors[k][-1] = 2
            # print(robot.learned_colors)

    if not robot.has_doll:
        robot.reverse_path = True
        robot.bounding_box = False
        robot.move_timed(how_long=1, direction="back", speed=1000)
        robot.rotate(170)
        return

    kp = 20
    ki = 1.5
    kd = 60.1
    black_counter = 0
    #robot.motors.alternative.run_forever(speed_sp=1000)
    can_break = False
    #contador_para_re = 40
    # limits
    pid = PID(kp, ki, kd, setpoint=77.4)

    # 83.3 indo
    # 75 voltando

    pid.output_limits = (-400, 400)
    while True:
        # print("Bouding box loop..")
        search = robot.sensor_data("ColorSensor")
        if search[0] == "Black" and search[1] == "Black":
            black_counter += 1
        if black_counter >= 50:
            drop_doll(robot)
            # move back with pid
            robot.move_timed(how_long=1.1, direction="back", speed=1000)
            robot.rotate(180)
            pid = PID(kp, ki, kd, setpoint=80.4)
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

        if can_break and robot.verifica_para_saida_do_bound_box() is True:
            robot.move_timed(how_long=1, direction="forward", speed=n_speed)
            break
    ev3.Sound.beep()
    return
