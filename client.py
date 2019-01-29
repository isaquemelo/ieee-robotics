#!/usr/bin/env python3

import paho.mqtt.client as mqtt


# This is the Subscriber

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("topic/test")


def on_message(client, userdata, msg):
    if msg.payload.decode() == "Hello world!":
        print("Yes!")
        ev3.Sound.beep()
        ev3.Sound.beep()
        ev3.Sound.beep()
        ev3.Sound.beep()
        ev3.Sound.speak("Message received.").wait()

        client.disconnect()


client = mqtt.Client()
client.connect("169.254.109.89", 1883, 60)

client.on_connect = on_connect
client.on_message = on_message

client.loop_forever()
