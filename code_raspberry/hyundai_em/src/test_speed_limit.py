#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import re
import requests
import time
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import NavSatFix

global speed_limit
speed_limit=0

def api_pub(result):	#send data to motor operating file
	motor_angle = rospy.Publisher('speed_limit_data',Int32MultiArray, queue_size =1)
	array_data= Int32MultiArray(data=[result])
	motor_angle.publish(array_data)

def tmap_api(msg):
    global speed_limit
    
    lon = 127.40019
    lat = 37.63589
    print(f"Latitude: {lat}, Longitude: {lon}")
    #return lon,lat
    version='1'
    UserAppKey = 'B1emXjw3LuA2918TPpvw5OtuEsUbBeuazEbCrsDi'
    url = 'https://apis.openapi.sk.com/tmap/road/nearToRoad?version='+version+'&appKey='+UserAppKey+'&lat='+str(lat)+'&lon='+str(lon)# API URL 설정
    print(url)
    response = requests.get(url)  # GET 요청
    data=response.text
    result=[]
    for i in re.split('[{,:"]',data):
        result.append(i)
    #print(result)
    a= result.index('speed')
    speed_limit=int(result[a+2])
    print("제한속도:",speed_limit,"Km/h")
    return speed_limit

rospy.init_node('speed_limit')
rospy.Subscriber('ublox_gps/fix', NavSatFix, tmap_api)
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    api_pub(speed_limit)
    rate.sleep()
