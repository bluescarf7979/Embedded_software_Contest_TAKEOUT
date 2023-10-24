#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import re
import requests
import time
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import NavSatFix

    

def api_pub(result):	#send data to motor operating file
	motor_angle = rospy.Publisher('speed_limit_data',Int32MultiArray, queue_size =1)
	array_data= Int32MultiArray(data=[result])
	motor_angle.publish(array_data)

def tmap_api(msg):
    lon = msg.longitude
    lat = msg.latitude
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
    api_pub(speed_limit)
    return speed_limit

while not rospy.is_shutdown():
    rospy.init_node('speed_limit')
    rospy.Subscriber('ublox_gps/fix', NavSatFix, tmap_api)
    rospy.spin()