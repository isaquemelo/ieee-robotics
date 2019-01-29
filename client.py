#!/usr/bin/env python3

import paho.mqtt.client as mqtt
import ev3dev.ev3 as ev3

# This is the Subscriber


def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("topic/test")


def on_message(client, userdata, msg):

    print("Yes!")
    ev3.Sound.beep()
    ev3.Sound.beep()
    ev3.Sound.beep()
    ev3.Sound.beep()
    ev3.Sound.speak("Message received.").wait()


client = mqtt.Client()
client.connect("169.254.20.22", 1883, 60)


client.on_connect = on_connect
client.on_message = on_message

client.loop_forever()
