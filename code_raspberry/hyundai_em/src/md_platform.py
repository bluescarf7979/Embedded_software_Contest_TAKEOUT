#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float32MultiArray, Int32MultiArray, String
import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
import time

global remember_temp,remember_rain,remember_snow,remember_speed_limit, snow_cover, black_ice

remember_temp=False
remember_rain=False
remember_snow=False
remember_speed_limit=False
snow_cover=False
black_ice=False

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {str(rc)}")
    client.subscribe("black_ice_data")
    client.subscribe("snow_speed_data")

def on_message(client, userdata, msg):
    global snow_cover, black_ice
    response=str(msg.payload.decode('utf-8'))
    if response.isdigit():
        snow_cover = int(response)
    if response == 'True':
        black_ice = True
    elif response == 'False':
        black_ice = False

def combined_callback(data):
    global remember_temp,remember_rain,remember_snow,remember_speed_limit
    # 메시지 유형 확인
    
    if isinstance(data, Float32MultiArray): 
        # Float32MultiArray 메시지 유형 처리
        a = data.data[0] #온도
        a1= data.data[1] #강수량
        a2= data.data[2] #전체 적설
    else:
        a = False
        
    if isinstance(data, Int32MultiArray):
        # Int32MultiArray 메시지 유형 처리
        b = data.data[0]
        #print("제한속도:", b)
    else:
        b= False

    if a is not False:
        remember_temp=a
        remember_rain=a1
        remember_snow=a2
        
    if b is not False:
        remember_speed_limit=b
    realmain() 
 
def listener():
    GPIO.setmode(GPIO.BCM)  # BCM 모드를 사용. 핀 번호가 아닌 GPIO 번호를 사용하여 핀을 지정.
    GPIO.setwarnings(False)
    LED_PIN = 17  # GPIO 핀 번호
    #GPIO.setup(LED_PIN, GPIO.OUT)

    rospy.init_node('result')

    # 두 토픽에 대한 하나의 구독자 및 콜백 함수 지정
    rospy.Subscriber("weather_data", Float32MultiArray, combined_callback)
    rospy.Subscriber("speed_limit_data", Int32MultiArray, combined_callback)
    #mqtt 사용
    broker_address = "192.168.0.84" #맞는 ip 주소로 바꾸기
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker_address, 1883, 60)
    client.loop_start()
    rospy.spin()

def realmain(): #진짜 메인문
    global remember_temp,remember_rain,remember_snow,remember_speed_limit, snow_cover, black_ice
    pub_state=rospy.Publisher('/mobility_state', String, queue_size=10)
    '''    0: 특이사항없음 1: 적설감지 2: 블랙아이스 감지
    remember_temp : 기상청 api 온도
    remember_rain : 기상청 api 강수량(여름철 1시간강수량, 겨울철 3시간 강수량)
    remember_snow : 기상청 api 적설량(전체 적설)
    remember_speed_limit : api 속도제한
    snow_cover : 적설량 <<int
    black_ice : 블랙아이스 유무 << False, True(블랙아이스 감지시)
    '''
    result = 0
    # 적설량 부분
    if snow_cover >0 and snow_cover is not False:
        if snow_cover >remember_snow-0.2*snow_cover or snow_cover <remember_snow+0.2*snow_cover:
            result =1        
    # 블랙아이스 부분
    if black_ice ==True:
        if remember_temp<=0 and remember_rain>0 or remember_snow>0:
            result = 2
    print('온도:',remember_temp)
    print('강수량:',remember_rain)
    print('제한속도:',remember_speed_limit)
    print('기상청 적설량:',remember_snow)
    print('적설량:',snow_cover) 
    print('블랙아이스 유무:',black_ice)

    LED_PIN = 17 
    if result==1 or result==2:
        pub_state.publish("slow_down")
        for i in range(50):
            #GPIO.output(LED_PIN, GPIO.HIGH)  # LED 켜기
            time.sleep(0.1)  # 1초 대기
            #GPIO.output(LED_PIN, GPIO.LOW)   # LED 끄기
            time.sleep(0.1)  # 1초 대기
    else:
        pub_state.publish("go")


    array_data= Int32MultiArray(data=[result,remember_speed_limit,snow_cover])
    pub=rospy.Publisher('result_data',Int32MultiArray,queue_size=10)


    pub.publish(array_data)
    
    return 0
    
    
    
if __name__ == '__main__':
    listener()


