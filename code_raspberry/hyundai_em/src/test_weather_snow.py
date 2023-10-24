#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import requests
import time
import math
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import NavSatFix
from datetime import datetime

def signal_handler(signal, frame):
    rospy.signal_shutdown("Ctrl+C pressed")

def haversine(lat1, lon1, lat2, lon2):
    R = 6371  # 지구 반지름 (킬로미터)

    # 각도를 라디안으로 변환
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    distance = R * c  # 거리 (킬로미터)
    return distance

def get_current_date_string():
    current_date = datetime.now().date()
    return current_date.strftime("%Y%m%d")

def get_current_hour_string():
    now = datetime.now()
    if now.hour==0:
        base_time = "2300"
    else:
        pre_hour = now.hour-1
        if pre_hour<10:
            base_time = "0" + str(pre_hour) + "00"
        else:
            base_time = str(pre_hour) + "00"
    if now.hour < 10:
        base_time = "0" + str(now.hour) + "00"
    else:
        base_time = str(now.hour) + "00"

    return base_time


def forecast(msg):
    lon = 127.40019
    lat = 37.63589
    keys = '1uzOYDl9QgmszmA5fZIJrQ'
    tm= '202212252300'
    stn=str(stn_name(lon,lat))
    authKey = keys
    url = 'https://apihub.kma.go.kr/api/typ01/url/kma_sfctm2.php?tm='+str(tm)+'&stn='+stn+'&authKey='+str(authKey)  # API URL 설정
    print(url)
    response = requests.get(url)  # GET 요청
    data=response.text
    result=data.split()
    a= result.index(tm)
    b= result.index('YYMMDDHHMI')
    c= result.index('IX')
    name=[]
    for i in range(b,c):
        name.append(result[i])
    for i in range(0,a):
        del result[0]
    del result[len(result)-1]
    for i in range (0,len(result)-1):
        print(name[i],result[i])
    for i in range (15,22,1):
        if float(result[i])<0:
            result[i]=0
    SD_HR3,SD_DAY,SD_TOT=float(result[11]),float(result[15]),float(result[21])
    # 응답 출력
    array_data= Float32MultiArray(data=[SD_HR3,SD_DAY,SD_TOT])
    pub=rospy.Publisher('weather_data',Float32MultiArray,queue_size=10)
    pub.publish(array_data)
    delay_time()
    return 0

def stn_name(longit,latit):
    city=[]
    tm= get_current_date_string()+get_current_hour_string()
    url = 'https://apihub.kma.go.kr/api/typ01/url/stn_inf.php?inf=SFC&stn=&tm='+str(tm)+'&help=1&authKey=DwUlcUFpRG6FJXFBaTRuPw' # API URL 설정
    response = requests.get(url)
    data = response.text
    result=data.split()
    a= result.index('90') #해당 url의 의미있는 첫번째 값이 90임. 만약 기상청의 데이터 내용 바뀔 시 수정.
    for i in range(0,a):
        del result[0]
    del result[len(result)-1]
    n = 0
    k = 0
    while True:
        try:
            if (result[15 * n+k]) == '----':
                k+=1
            city_data = [
                int(result[15 * n+k]),
                result[15 * n + k+10],
                float(result[15 * n + k+1]),
                float(result[15 * n + k+2])
            ]
            city.append(city_data)
            n += 1
        except IndexError:
            break
    #print(city)
    target_lon = longit
    target_lat = latit
    closest_station = None
    min_distance = float('inf')

    for station in city:
        station_id, _, station_lon, station_lat = station
        distance = haversine(target_lat, target_lon, station_lat, station_lon)

        if distance < min_distance:
            min_distance = distance
            closest_station = station

    
    return closest_station[0]

def delay_time():
    i=0
    while i<2:
        rospy.sleep(1)
        i+=1
        if rospy.is_shutdown():
            break

if __name__ == "__main__":
    rospy.init_node('weather')
    rospy.Subscriber('ublox_gps/fix', NavSatFix, forecast)
    rospy.spin()
    
