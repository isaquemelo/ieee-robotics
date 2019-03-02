#!usr/bin/python3

import ev3dev.ev3 as ev3
import math
from datetime import datetime, timedelta
import time
import paho.mqtt.client as mqtt
from struct import *
from simple_pid import PID


def map_values(n, start1, stop1, start2, stop2):
    return ((n - start1) / (stop1 - start1)) * (stop2 - start2) + start2


class Duo:
    def __init__(self, sensor_left, sensor_right, sensor_back=None):
        self.left = sensor_left
        self.right = sensor_right

        if sensor_back is not None:
            self.back = sensor_back

        self.values = (self.left, self.right)


DEFAULT_SPEED = 400


class Robot:
    ev3.Sound.speak("Robot started...")

    def __init__(self):
        # define sensors
        self.gyroscope_sensor = ev3.GyroSensor('in1')
        self.gyroscope_sensor.mode = 'GYRO-ANG'
        self.color_sensors = Duo(ev3.ColorSensor('in2'), ev3.ColorSensor('in3'))
        # self.ultrasonic_sensors = ev3.UltrasonicSensor('in4')

        # define motors
        self.motors = Duo(ev3.LargeMotor('outA'), ev3.LargeMotor('outB'))
        self.handler = ev3.LargeMotor('outC')

        # define status
        self.in_rect = False
        self.rect_color = "Undefined"

        # define position
        self.infrared_sensors = (0, 0)

        self.white_counter = 0

        # coisas para identificar o final da pista
        self.plataforma_de_entrega_entrado = None
        self.fila_para_registro_do_fim = ["White", "White"]
        # contador de tempo para o identificar de fim de pista
        self.kon = 14
        self.wait_until_activate_kon = True

    def update_no_status_de_registro_de_fim_do_percursso(self):
        # adiciona e deleta elemento da fila que registara entrada e sainda do robo da plataforma (if 1.0)
        # pra o lado esquerdo
        # cores_sensores = self.sensor_data("ColorSensor").copy()
        if self.sensor_data("ColorSensor")[0] not in ["Black", "Undefined", "Brown"] and \
                self.fila_para_registro_do_fim[-1] != self.sensor_data("ColorSensor")[0]:
            del self.fila_para_registro_do_fim[0]
            self.fila_para_registro_do_fim.append(self.sensor_data("ColorSensor")[0])
            #print("Entrou no if pelo lado esquerdo que armazena na fila = {}".format(self.fila_para_registro_do_fim))
        # pra o lado direito
        elif self.sensor_data("ColorSensor")[1] not in ["Black", "Undefined", "Brown"] and \
                self.fila_para_registro_do_fim[-1] != self.sensor_data("ColorSensor")[1]:
            del self.fila_para_registro_do_fim[0]
            self.fila_para_registro_do_fim.append(self.sensor_data("ColorSensor")[1])
            #print("Entrou no if pelo lado pra direito que armazena na fila = {}".format(self.fila_para_registro_do_fim))

        # (if 1.0) registra se o robo esta entrando ou saindo na plataforma de entraga
        if self.kon > 13:
            self.wait_until_activate_kon = True

        if "White" not in self.fila_para_registro_do_fim and self.wait_until_activate_kon == True:
            #print("---------------------------------------------------------------------------------------------------")
            if self.plataforma_de_entrega_entrado in [None, "saindo"]:
                self.plataforma_de_entrega_entrado = "entrando"
            elif self.plataforma_de_entrega_entrado in [None, "entrando"]:
                self.plataforma_de_entrega_entrado = "saindo"

            self.kon = 0
            self.wait_until_activate_kon = False

        # ou seja so ira realizar a verificacao depois de um certo tempo e seta a variavel de interesse corretamente
        # este contador foi realmente necessario posi nao foi possivel fazer a fila armazenar as 3 cores do fim de pista
        # por isso teve que ser feito uma fila que armazenace apenas dois so as vezes o robo pela as 3 cores
        # (as vezes por causa do color realigment que nao podia ser retirado) e ai
        # comutava entre entrando e saindo ao mesmo tempo isso fazia a self.plataforma_de_entrega_entrando ficar louca
        # so que com o contador para restringir a comutacao desta ele ficou bem melhor mesmo o robo identificando as 3 ou as
        # duas cores do fim da pista ele so comutara a variavel na hora certa
        if self.wait_until_activate_kon == False:
            if self.kon <= 13:
                self.kon += 1



        # parte de debugg
        #print("self.plataforma_de_entrega = {}".format(self.plataforma_de_entrega_entrado))
        if self.plataforma_de_entrega_entrado == None:
            print("NOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOONE")
        elif self.plataforma_de_entrega_entrado == "entrando":
            print("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<ENTRANDO")
        elif self.plataforma_de_entrega_entrado == "saindo":
            print("}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}SAINDO")
        print("fila = {}".format(self.fila_para_registro_do_fim))


    def update(self):
        # dis se o robo esta entrando ou saindo na plataforma de entrega
        self.update_no_status_de_registro_de_fim_do_percursso()
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


def undefined_dealing(color_sensor):
    sensor_color = color_sensor
    if sensor_color[0] == "Undefined" or sensor_color[1] == "Undefined":
        if sensor_color[0] == "Undefined":
            robot.move_timed(how_long=0.6, direction="back")
            robot.rotate(30, axis="diferente")
        elif sensor_color[1] == "Undefined":
            robot.move_timed(how_long=0.6, direction="back")
            robot.rotate(-30, axis="diferente")


last_same_color = [None, ""]
color = 0
realignment_counter = 0
rect_check = False

# 15.6 0 4.8
pid = PID(15.6, 0, 4.8, setpoint=-4)


def color_realignment(robot, color_sensor_data, infrared_sensor, move_forward=True, speed=DEFAULT_SPEED):
    robot.update()
    global last_same_color, color, realignment_counter, rect_check
    reverse = False

    search = color_sensor_data


    if robot.in_rect:
        if (search[0] != robot.rect_color) or (search[1] != robot.rect_color):
            # print("Saiuuuuuuu\n\n\n")
            robot.rect_color = "Undefined"
            robot.in_rect = False
            return None

    if move_forward:
        if search[0] == search[1]:
            pid.output_limits = (-600, 600)
            control = pid(robot.infrared_sensors[1] - robot.infrared_sensors[0])
            n_speed = 600

            if control > 400:
                control = 400
            if control < -400:
                control = -400

            # n_speed = 400
            #
            # if control > 600:
            #     control = 600
            # if control < -600:
            #     control = -600

            if robot.sensor_data("ColorSensor")[0] != "White" and robot.sensor_data("ColorSensor")[1] != "White":
                robot.motors.left.run_forever(speed_sp=speed)
                robot.motors.right.run_forever(speed_sp=speed)
            else:
                robot.motors.left.run_forever(speed_sp=n_speed + control)
                robot.motors.right.run_forever(speed_sp=n_speed - control)

            last_same_color = search

            if search[1] not in ["White", "Undefined", "Brown", "Black"]:
                color += 1

            if color > 13:
                if search[0] in ["White", "Undefined"] or search[1] in ["White", "Undefined"]:
                    color = 0

                    robot.motors.left.stop()
                    robot.motors.right.stop()

                    robot.move_timed(how_long=0.4, direction="back")

                    # ev3.Sound.speak("Robot aligned...").wait()
                    robot.in_rect = True
                    #time.sleep(3) Tempo para verificar se ele está voltando de fato para dentro do quadrado com os sensores de cor.
                    if robot.sensor_data("ColorSensor")[0] not in ["White", "Undefined"]:
                        robot.rect_color = robot.sensor_data("ColorSensor")[0]
                    elif robot.sensor_data("ColorSensor")[1] not in ["White", "Undefined"]:
                        robot.rect_color = robot.sensor_data("ColorSensor")[1]


                    #print("Estou num quadrado: ", robot.rect_color)
                    rect_check = True
                    # ev3.Sound.speak("The rect color is " + robot.rect_color).wait()
                    return "On square"

    if last_same_color[0] == "White" and last_same_color[1] == "White":
        reverse = True
    else:
        reverse = False

    if search[0] == "White" and search[1] != "White":

        robot.motors.left.stop()
        robot.motors.right.stop()

        while search[0] != search[1]:
            if reverse:
                robot.motors.left.run_forever(speed_sp=speed)
            else:
                robot.motors.right.run_forever(speed_sp=speed)

            search = robot.sensor_data("ColorSensor")
            undefined_dealing(search)

        time.sleep(0.15)
        robot.move_timed(direction="back")
        realignment_counter += 1

        if realignment_counter > 7:
            ev3.Sound.speak("Robot has exceed correction numbers...").wait()
            realignment_counter = 0
            robot.move_timed(direction="forward", how_long=0.6)

        if reverse:
            robot.motors.left.stop()
        else:
            robot.motors.right.stop()

    elif search[0] != "White" and search[1] == "White":
        robot.motors.left.stop()
        robot.motors.right.stop()

        while search[0] != search[1]:
            if reverse:
                robot.motors.right.run_forever(speed_sp=speed)
            else:
                robot.motors.left.run_forever(speed_sp=speed)

            search = robot.sensor_data("ColorSensor")
            undefined_dealing(search)

        time.sleep(0.15)
        robot.move_timed(direction="back")
        realignment_counter += 1

        if realignment_counter > 7:
            ev3.Sound.beep().wait()
            ev3.Sound.beep().wait()
            realignment_counter = 0
            robot.move_timed(direction="forward", how_long=0.6)

        if reverse:
            robot.motors.right.stop()
        else:
            robot.motors.left.stop()


def return_last_color(robot, square_color, last_choice):
    robot.rotate(180)
    while True:
        robot.update()
        search = robot.sensor_data("ColorSensor")
        result = color_realignment(robot, search, robot.infrared_sensors)

        if result == "On square" and robot.rect_color == square_color:
            #print("TERMNOU IF")
            robot.in_rect = True
            break

    #print("\n\nEntregue\n\n")


robot = Robot()

client = mqtt.Client()
client.connect("169.254.173.223", 1883, 60)


def on_message(client, userdata, message):
    carga = unpack("iid", message.payload)
    robot.infrared_sensors = carga[:2]
    # print("Received message:", carga[:2], time.time() - float(carga[2]))


def on_connect(client, userdata, flags, rc):
    #print("The robots are connected with result code", str(rc))
    client.subscribe("topic/sensors")


client.on_connect = on_connect
client.on_message = on_message

client.loop_start()

def main():
    try:
        learned_colors = {'Green': 'forward', 'Red': 'left', 'Blue': 'forward'}
        learned_colors = {}

        being_learned = "Undefined"

        # learning_dic = {"Green": ["left", "right", "forward"]}
        learning_dic = {}
        im_learning = False
        result = None

        while True:
            # print("LOOP PRIMARIO")
            # print("TA APRENDENDO = {}".format(im_learning))
            robot.update()
            search = robot.sensor_data("ColorSensor")
            result = color_realignment(robot, search, robot.infrared_sensors)
            white_counter = 0

            if result == "On square" or robot.in_rect:
                #print("On square")
                # if search[0] != being_learned and search[0] not in ["White", "Undefined"]:
                #     learned_colors[being_learned] = learning_dic[being_learned][0]
                #     im_learning = False
                #     being_learned = "Undefined"
                #     print("Aprendi uma nova cor, segue o dicionario:", learned_colors)

                if not im_learning:
                    # print("SE O ROBO DEU O BUG ESPERADO JA SEI QUE TEM QUE MEXER COM A robot.rect_colors")
                    if robot.rect_color not in ["White", "Undefined", "Black"]:
                        # print("ENTROU NO IF LOGO NAO DEU O BUG")
                        try:
                            #print("Tentando executar acao para a cor:", robot.rect_color)
                            #print("Aprendidos ate agora:", learned_colors)

                            if learned_colors[robot.rect_color] and robot.sensor_data("ColorSensor") not in ["Black", "Brown", "Undefined"]:
                                robot.run_action(learned_colors[robot.rect_color], im_learning)
                                robot.move_timed(how_long=0.4)
                                color = 0
                                time.sleep(0.5)

                        except:
                            # print("ENTROU NO EXCEPT")
                            #print("Acao para a cor:", robot.rect_color, "nao existe ou falhou!")
                            being_learned = robot.rect_color
                            learning_dic[being_learned] = ["right", "forward", "left"]
                            im_learning = True

                elif im_learning:
                    robot.run_action(learning_dic[being_learned][0], im_learning)
                    while True:
                        # print("LOOP SECUNDARIO")
                        robot.update()
                        search = robot.sensor_data("ColorSensor")
                        result = color_realignment(robot, search, robot.infrared_sensors)


                        if robot.sensor_data("ColorSensor")[0] == robot.sensor_data("ColorSensor")[1] and robot.sensor_data("ColorSensor")[1] == "White":
                            white_counter += 1

                        print(white_counter)

                        # print("On square loop secundario")
                        if robot.sensor_data("ColorSensor")[0] == robot.sensor_data("ColorSensor")[1] and \
                                robot.sensor_data("ColorSensor")[0] not in ["White", "Undefined", "Black"]:
                            if robot.sensor_data("ColorSensor")[0] != being_learned or (robot.sensor_data("ColorSensor")[0] == being_learned and white_counter >= 5):
                                learned_colors[being_learned] = learning_dic[being_learned][0]
                                im_learning = False
                                being_learned = "Undefined"
                                learning_dic = {}
                                #print("Aprendi uma nova cor, segue o dicionario:", learned_colors)
                                break

                        if search[0] == "Black" and search[1] == "Black":
                            #print("DIRECAO ERRADA")
                            global realignment_counter
                            robot.motors.left.stop()
                            robot.motors.right.stop()
                            time.sleep(0.3)
                            realignment_counter = 0

                            last_choise = learning_dic[being_learned][0]
                            del learning_dic[being_learned][0]

                            return_last_color(robot, being_learned, last_choise)

                            #print("learned_dic =", learning_dic)
                            break


                        """
                        if len(learning_dic[being_learned]) == 1:
                            learned_colors[being_learned] = learning_dic[being_learned][0]
                            im_learning = False
                            being_learned = "Undefined"
                            learning_dic = {}
                            cooisa = False
                            print("Aprendi uma nova cor, segue o dicionario (POR EXCLUSSAO):", learned_colors)

                            print("learned_dic = {}".format(learning_dic))
                            break
                        """

                    white_counter = 0

            if search[0] == "Undefined" and search[1] == "Undefined":
                robot.motors.left.stop()
                robot.motors.right.stop()


    except KeyboardInterrupt:
        robot.motors.right.stop()
        robot.motors.left.stop()
        client.loop_stop()
        client.disconnect()



if __name__ == '__main__':
    main()