#!/usr/bin/env python3
from .duo import Duo
import ev3dev.ev3 as ev3
import math
from datetime import datetime, timedelta
import json

DEFAULT_SPEED = 400


def map_values(n, start1, stop1, start2, stop2):
    return ((n - start1) / (stop1 - start1)) * (stop2 - start2) + start2


class Robot:
    ev3.Sound.speak("Robot started...")

    def __init__(self):
        # define sensors
        self.gyroscope_sensor = ev3.GyroSensor('in3')
        self.gyroscope_sensor.mode = 'GYRO-RATE'
        self.gyroscope_sensor.mode = 'GYRO-ANG'
        self.DEFAULT_SPEED = DEFAULT_SPEED
        self.color_sensors = Duo(ev3.ColorSensor('in1'), ev3.ColorSensor('in2'))
        self.ultrasonic_sensor = ev3.UltrasonicSensor('in4')
        self.ta_no_final_da_pista = False

        self.time_desabilita_o_realinhamento_da_cor = datetime.now()

        # define motors
        self.motors = Duo(ev3.LargeMotor('outA'), ev3.LargeMotor('outD'), ev3.LargeMotor('outC'))
        # self.handler = ev3.LargeMotor('outC')
        # self.learned_colors = {}
        # self.learned_colors = {'Blue': ['left'], 'Red': ['forward'], 'Green': ['right']}
        self.primeiro_bounding_box = True
        # define status
        self.historic = [""]
        self.undefined_counter = 0
        self.in_rect = False
        self.rect_color = "Undefined"
        self.reverse_path = None
        self.dor_open = True
        self.has_doll = False    # OBS: LEMBRAR DE SETAR PRA FALSE
        self.done_learning = False
        self.voltou = False
        self.tempo_para_chamar_run_action = datetime.now()
        self.has_came_from_json = False

        self.bounding_box = False

        # self.historic = ['', 'left', 'forward', 'right', 'right', 'forward', 'left']
        #self.reverse_path = True
        # self.dor_open = True
        #self.has_doll = True
        # self.done_learning = True

        self.nao_pode = False
        self.realigment_counter = 0
        self.starting_angle = self.sensor_data("GyroSensor")

        # define network sensors

        self.infrared_sensors = (0, 0)
        self.white_counter = 0

        # path detection variables
        self.fila_para_registro_do_fim = ["White", "White"]
        # contador de tempo para o identificar de fim de pista
        self.kon_const = 25
        self.kon = self.kon_const + 1

        self.file_name = "learned_colors.json"
        self.fixed_file_name = "learning_dic.json"
        # tenta abrir arquivo json com cores aprendidas
        try:
            with open(self.file_name) as json_file:
                process = json.load(json_file)
                self.learned_colors = process[0]

        except FileNotFoundError:
            self.learned_colors = {}

        # tenta abrir achar arquivo de aprendizado individual, se nao existir cria:
        try:
            with open(self.fixed_file_name) as json_file:
                process = json.load(json_file)

        except FileNotFoundError:
            with open(self.fixed_file_name, 'w') as outfile:
                json.dump({}, outfile)



    def voltou_inicio_pista(self):
        search = self.sensor_data("ColorSensor")

        if search[0] == "Undefined" and search[1] != "Undefined":  # so o da esquerda  ta fora da pista
            while True:
                search = self.sensor_data("ColorSensor")
                if search[0] == "White":
                    self.stop_motors()
                    break
                else:
                    self.motors.left.run_forever(speed_sp=-500)
        elif search[1] == "Undefined" and search[0] != "Undefined":  # so o da direita ta fora da pista
            while True:
                search = self.sensor_data("ColorSensor")
                if search[1] == "White":
                    self.stop_motors()
                    break
                else:
                    self.motors.right.run_forever(speed_sp=-500)
        elif search[1] == "Undefined" and search[0] == "Undefined":   # os dois estao fora da pista ao mesmo tempo
            while True:
                search = self.sensor_data("ColorSensor")
                if search[1] == "White" and search[0] == "White":
                    self.stop_motors()
                    break
                else:
                    self.motors.right.run_forever(speed_sp=-500)
                    self.motors.left.run_forever(speed_sp=-500)
        self.move_timed(how_long=0.2, direction="back")
        return


    def update_no_status_de_registro_de_fim_do_percursso(self):
        # adiciona e deleta elemento da fila que registara entrada e sainda do robo da plataforma (if 1.0)
        # pra o lado esquerdo
        # cores_sensores = self.sensor_data("ColorSensor").copy()
        cor_do_sensor = self.sensor_data("ColorSensor")
        if cor_do_sensor[0] not in ["Black", "Undefined", "Brown"] and \
                self.fila_para_registro_do_fim[-1] != cor_do_sensor[0]:
            del self.fila_para_registro_do_fim[0]
            self.fila_para_registro_do_fim.append(cor_do_sensor[0])
            # print("Entrou no if pelo lado esquerdo que armazena na fila = {}".format(self.fila_para_registro_do_fim))
        # pra o lado direito

        elif cor_do_sensor[1] not in ["Black", "Undefined", "Brown"] and \
                self.fila_para_registro_do_fim[-1] != cor_do_sensor[1]:
            del self.fila_para_registro_do_fim[0]
            self.fila_para_registro_do_fim.append(cor_do_sensor[1])
            # print("Entrou no if pelo lado pra direito que armazena na fila = {}".format(self.fila_para_registro_do_fim))

        # (if 1.0) registra se o robo esta entrando ou saindo na plataforma de entraga
        if self.kon > self.kon_const:
            if self.verifica_para_bound_box() == True:
                # print("---------------------------------------------------------------------------------------------------")
                if self.reverse_path in [True, None]:
                    self.reverse_path = False
                else:
                    self.reverse_path = True
                self.bounding_box = True
                self.kon = 0

        elif self.kon <= self.kon_const:
            self.kon += 1

    def verifica_para_bound_box(self):
        for i in self.fila_para_registro_do_fim:
            if i in ["Black", "Undefined", "Brown", "White"]:
                return False
        return True

    def verifica_para_saida_do_bound_box(self):
        for i in self.sensor_data("ColorSensor"):
            if i in ["Black", "White", "Undefined", "Brown"]:
                return False
        self.bounding_box = False
        #print("SAINDO do bound box")
        #print("valoes na fila de igentificassao de fim de pista: {}".format(self.fila_para_registro_do_fim))
        self.reverse_path = True
        return True

    def update(self):
        # sensors update
        if self.color_sensors.left.color == 6 and self.color_sensors.right.color == 6:
            self.white_counter += 1

        if self.white_counter > 15:
            self.nao_pode = False
            self.realigment_counter = 0
            self.white_counter = 0
        # history update
        # position update
        self.update_no_status_de_registro_de_fim_do_percursso()
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
                4: 'Undefined', # era Yellow, para evitar cores que não exitem e acabar chamando o bounding box quando não deve
                5: 'Red',
                6: 'White',
                7: 'Brown'
            }

            # print([dict_colors[self.color_sensors.left.color], dict_colors[self.color_sensors.right.color]])
            return [dict_colors[self.color_sensors.left.color], dict_colors[self.color_sensors.right.color]]

    def rotate(self, angle, axis="own", speed=DEFAULT_SPEED):

        if 30 > angle > 0:
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
        self.starting_angle = self.sensor_data("GyroSensor")

    def move_timed(self, how_long=0.3, direction="forward", speed=DEFAULT_SPEED):
        end_time = datetime.now() + timedelta(seconds=how_long)

        vel = speed

        if direction != "forward":
            vel = -speed

        # print("Starting time!")
        while datetime.now() < end_time:
            self.motors.left.run_forever(speed_sp=vel), self.motors.right.run_forever(speed_sp=vel)
        # print("Time is over!")
        self.motors.left.stop(), self.motors.right.stop()

    def learned_colors_is_empity(self):
        # print("CHAMOU A FUNÇÃO")
        keys = list(self.learned_colors.keys())
        # print("learned_colors = {}".format(self.learned_colors))
        for k in keys:
            # if type(self.learned_colors[k][-1]) != int:
            #     print("Retornou String")
            #     return "tentou chamar pra uma String"
            # print("valor = {}".format(self.learned_colors[k][-1]))
            if self.learned_colors[k][-1] >= 1:
                # print("Retornou False")
                return False
        # print("Retornou True")
        return True

    def run_action(self, direction, still_learning=True):
        print("Run action chamada com os seguintes paramentros:", "direction:", direction, "still_learning:", still_learning)
        # print("CHAMOU O RUN_ACTION COM O REVERSE_PATH = {}".format(self.reverse_path))
        self.realigment_counter = 0
        # if self.nao_pode:
        #     self.move_timed(how_long=0.2, direction="back")
        #     return

        if datetime.now() < self.tempo_para_chamar_run_action:
            # print("CHAMOU A RUN_ACTION NO TEMPO ERRADO")
            ev3.Sound.beep()
            ev3.Sound.beep()
            self.move_timed(how_long=0.2, direction="back")
            return
        else:
            # print("CHAMOU A RUN_ACTION NO TEMPO CERTO")
            self.tempo_para_chamar_run_action = datetime.now() + timedelta(seconds=5)

        if self.reverse_path is True:
            self.learned_colors[self.rect_color][-1] -= 1
            # print("EXECUTOU O DESCARREGAMENTO DA COR")
            print(self.learned_colors)
            self.stop_motors()
            # if self.ta_no_final_da_pista is True:
            #     self.ta_no_final_da_pista = False
            #     self.rotate(180)

            if self.primeiro_bounding_box is False:
                if self.voltou is False:
                    if self.learned_colors_is_empity() is True:
                        self.ta_no_final_da_pista = True
                        self.rotate(180)
                        self.reverse_path = False

        self.nao_pode = True

        # if self.reverse_path:
        #     if not still_learning:
        #         if direction == "forward":
        #             pass
        #         elif direction == "left":
        #             self.rotate(90, axis="own")
        #         elif direction == "right":
        #             self.rotate(-90, axis="own")
        #     else:
        #         self.rotate(-90, axis="own")
        #     # return None
        #
        # else:
        #     if not still_learning:
        #         if direction == "forward":
        #             pass
        #         elif direction == "left":
        #             self.rotate(-90, axis="own")
        #         elif direction == "right":
        #             self.rotate(90, axis="own")
        #     else:
        #         if :
        #             self.rotate(90, axis="own")
        #         else:
        #             self.rotate(90, axis="own")
        #     # return None

        if not still_learning:
            if direction == "forward":
                pass
            elif direction == "left":
                self.rotate(90 if self.reverse_path else -90, axis="own")
            elif direction == "right":
                self.rotate(-90 if self.reverse_path else 90, axis="own")
        else:
            if self.has_came_from_json:
                if direction == "forward":
                    pass
                elif direction == "left":
                    self.rotate(-90, axis="own")
                elif direction == "right":
                    self.rotate(90, axis="own")

                self.has_came_from_json = False

            else:
                self.rotate(90, axis="own")



        # return None



    def stop_motors(self):
        self.motors.left.stop()
        self.motors.right.stop()
