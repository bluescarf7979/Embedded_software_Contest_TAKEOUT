#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import paho.mqtt.client as mqtt
import math  

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {str(rc)}")

def depth_callback(data):
    estimated_depth = data.data
    rounded_depth = round(estimated_depth)      
    client.publish("snow_speed_data", str(rounded_depth))  

client = mqtt.Client()
client.on_connect = on_connect
client.connect("MQTT_BROKER_IP", 1883, 60) 
client.loop_start()

rospy.init_node('mqtt_depth_bridge', anonymous=True)
rospy.Subscriber("estimated_depth", Float64, depth_callback)
rospy.spin()

