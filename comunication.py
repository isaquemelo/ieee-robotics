import paho.mqtt.client as mqtt
import ev3dev.ev3 as ev3

client = mqtt.Client()
client.connect("localhost", 1883, 60)


while True:
    if input() == "enviar":
        client.publish("topic/test", "Hello world!")
        ev3.Sound.speak("Message sent").wait()

    else:
        break

client.disconnect()
