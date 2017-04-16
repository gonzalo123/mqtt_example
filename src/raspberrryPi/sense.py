import paho.mqtt.client as mqtt
from sense_hat import SenseHat

sense = SenseHat()
sense.clear()
mqttServer = "192.168.1.104"

red = [255, 0, 0]
green = [0, 255, 0]
yellow = [255, 255, 0]
black = [0, 0, 0]

def on_connect(client, userdata, rc):
    print("Connected!")
    client.subscribe("potentiometer")

def on_message(client, userdata, msg):
    value = (64 * int(msg.payload)) / 100
    O = black
    if value < 21:
        X = red
    elif value < 42:
        X = yellow
    else:
        X = green

    sense.set_pixels(([X] * value) + ([O] * (64 - value)))

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(mqttServer, 1883, 60)
client.loop_forever()