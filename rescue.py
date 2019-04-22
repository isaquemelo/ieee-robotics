#usr/bin/env python3
import ev3dev.ev3 as ev3
from assets.classes.robot import Robot
from simple_pid import PID
from datetime import datetime, timedelta
import time
import json

DEFAULT_SPEED = 400


def deal_ret(robot):
    if robot.reverse_path is True:
        robot.rotate(90, speed=300)
    elif robot.reverse_path in [False, None]:
        robot.rotate(-90, speed=300)
    robot.has_doll = True
    robot.reverse_path = False
    return


def rescue(robot, speed=DEFAULT_SPEED):
    if robot.time_desabilita_o_realinhamento_da_cor > datetime.now():  # garanti que o robo nao tente resgata na saida da bounding box
        print("TENTOU RESGATAR QUANDO TAVA SAINDO DA BOUNDING BOX")
        return

    # garanti que o robo nao tente resgatar na situassao da foto 1,2
    # search = robot.sensor_data("ColorSensor")
    # if search[0] in ["Blue", "Red", "Green"] or search[1] in ["Blue", "Red", "Green"] \
    #         or robot.infrared_sensors[0] > 20:
    #     print("PREVINIU UM RESGATE DO LADO ERRADO DA PLATAFORMA")
    #     return
    """
    FOI PARA O TESTE DO ERRO DA FOTO 1,2
    else:
        print("ERRO")
        ev3.Sound.beep()
        return
    """
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

                while True:  # resgate seguro
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
                    # robot.rotate(-9) # OBS: descomentar se for necessario
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
                    while True:  # resgate ariscado pelo lado esquerdo
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
                    # robot.rotate(9) # OBS: descomentar se for necessario
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
                    while True:  # resgate ariscado pelo lado direito
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
    robot.stop_motors()
    if robot.has_doll:
        ev3.Sound.beep()
        ev3.Sound.beep()
        robot.motors.alternative.stop()
        robot.motors.alternative.run_timed(time_sp=2000, speed_sp=-1000)
        robot.stop_motors()
        robot.dor_open = False
        robot.has_doll = False


def bounding_box(robot, speed=DEFAULT_SPEED):
    robot.stop_motors()
    he_pra_retornar = False
    if not robot.has_doll:
        robot.reverse_path = True
        robot.bounding_box = False
        robot.move_timed(how_long=1, direction="back", speed=1000)
        robot.rotate(180, speed=1000, axis="own")
        he_pra_retornar = True

    for i in range(3):
        ev3.Sound.beep()
        print("cores que chamaram a bounding box = {}".format(robot.fila_para_registro_do_fim))
    # print("sel.kon = {}".format(robot.kon))
    robot.kon = 0

    #print("CHAMADO")
    for i in range(3):
        ev3.Sound.beep()
        #print(robot.fila_para_registro_do_fim)

    robot.voltou = False
    # recarrega todos as cores pelas quais vai passar voltando;
    if robot.primeiro_bounding_box is True:
        with open(robot.file_name, 'w') as outfile:
            json.dump([robot.learned_colors, True], outfile)


        robot.primeiro_bounding_box = False
        # print("PRIMEIRA CARGA")
        keys = robot.learned_colors.keys()
        for k in keys:
            robot.learned_colors[k][-1] = 2
            print(robot.learned_colors)
    else:
        # print("CARGA 1 + N")
        for k in sorted(robot.learned_colors.keys()):
            robot.learned_colors[k][-1] = 2
            # print(robot.learned_colors)

    # if not robot.has_doll:
    #     robot.reverse_path = True
    #     robot.bounding_box = False
    #     robot.move_timed(how_long=1, direction="back", speed=1000)
    #     robot.rotate(180, speed=1000, axis="own")
    #     return
    if he_pra_retornar is False:
        kp = 40
        ki = 0
        kd = 60.04
        black_counter = 0
        can_break = False
        #contador_para_re = 40
        # limits
        li = 76.6
        pid = PID(kp, ki, kd, setpoint=li)
        n_speed = 350

        # 83.3 indo
        # 75 voltando


        pid.output_limits = (-400, 400)
        while True:
            #print("Bouding box loop..")
            ultrasonico = robot.sensor_data("Ultrasonic")

            # print(robot.infrared_sensors[0])
            if 50 <= ultrasonico <= 100:
                control = pid(robot.sensor_data("Ultrasonic"))
                if control > 500:
                    control = 500
                if control < -500:
                    control = -500

                robot.motors.left.run_forever(speed_sp=n_speed + control)
                robot.motors.right.run_forever(speed_sp=n_speed - control)
            else:
                robot.motors.left.run_forever(speed_sp=n_speed+50)  # coloquei mais 50 para o robo nao engachar o lado esquerdo na hora de entrar na bounding box
                robot.motors.right.run_forever(speed_sp=n_speed)

            search = robot.sensor_data("ColorSensor")
            if search[0] == "Black" and search[1] == "Black":
                black_counter += 1

            if black_counter >= 60:

                drop_doll(robot)
                # move back with pid
                robot.move_timed(how_long=1.1, direction="back", speed=1000)
                black_counter = 0
                can_break = True
                robot.stop_motors()
                robot.rotate(90, axis = "own", speed=1000)
                robot.move_timed(how_long=7, speed=450)
                robot.move_timed(how_long=0.3, direction="back", speed=800)
                robot.rotate(90, axis = "own", speed=1000)
                # robot.move_timed(how_long=0.3, direction="back", speed=800)
                robot.move_timed(how_long=5, direction="forward", speed=450)
                robot.move_timed(how_long=0.2, direction="back", speed=800)
                robot.rotate(90, axis = "own", speed=1000)

                # coisas para verificação de sainda pelo angulorobot
                robot.gyroscope_sensor.mode = 'GYRO-RATE'
                robot.gyroscope_sensor.mode = 'GYRO-ANG'
                # coisas para verificação de sainda pelo angulorobot

                # end_time = datetime.now() + timedelta(seconds=7)
                pid = PID(kp, ki, kd, setpoint=1.3)
                # while datetime.now() < end_time:
                while True:
                    #print("DENTRO DO LOOP MAIS INTERNO")
                    n_speed = 350
                    # print("ULTRASONICO: ", ultrasonico)

                    # verifica para sair basiado na cor
                    search = robot.sensor_data("ColorSensor")
                    # print("SEARCH = {}".format(search))
                    if search[1] in ["Green", "Red", "Blue"]:
                        robot.stop_motors()
                        print("VOLTOU PELA COR = {}".format(search[1]))
                        robot.time_desabilita_o_realinhamento_da_cor = datetime.now() + timedelta(seconds=5) # PARA EVITAR QUE O ROBO CHAME TENTE RALINHAR COR A COR NA SAIDA DA BOUNDING BOX
                        # print("TIME QUANDO SAI DA BOUNDING BOX = {}".format(robot.time_desabilita_o_realinhamento_da_cor))
                        ev3.Sound.beep().wait()
                        robot.reverse_path = True
                        robot.bounding_box = False
                        robot.ta_na_ranpa = False
                        return

                    # verifica para sair basiado no angulo
                    if robot.sensor_data('GyroSensor') <= -90:
                        robot.stop_motors()
                        print("VOLTOU PELO ANGULO = {}".format(robot.sensor_data('GyroSensor')))
                        #robot.move_timed(how_long=1.2, speed=300)
                        robot.time_desabilita_o_realinhamento_da_cor = datetime.now() + timedelta(seconds=5)  # PARA EVITAR QUE O ROBO CHAME TENTE RALINHAR COR A COR NA SAIDA DA BOUNDING BOX
                        # print("TIME QUANDO SAI DA BOUNDING BOX = {}".format(robot.time_desabilita_o_realinhamento_da_cor))
                        robot.reverse_path = True
                        ev3.Sound.beep().wait()
                        robot.bounding_box = False
                        robot.ta_na_ranpa = False
                        return

                    control = pid(robot.infrared_sensors[0])
                    #print(robot.infrared_sensors[0])
                    if control > 600:
                        control = 600
                    if control < -600:
                        control = -600

                    robot.motors.left.run_forever(speed_sp=n_speed + control)
                    robot.motors.right.run_forever(speed_sp=n_speed - control)

                robot.rotate(90)


                # exit()

            #control = pid(robot.sensor_data("Ultrasonic"))
            #ultrasonico = robot.sensor_data("Ultrasonic")



        #     if can_break and robot.verifica_para_saida_do_bound_box() is True:
        #         robot.move_timed(how_long=1, direction="forward", speed=n_speed)
        #         robot.stop_motors()
        #         break
        # ev3.Sound.beep()
        # return
    robot.ta_na_ranpa = False
    return