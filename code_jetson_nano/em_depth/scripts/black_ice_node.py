#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import struct
import paho.mqtt.client as mqtt
from time import sleep

# 전역 변수 설정
outlier_points_data = []
outlier_depth_cloud_data = []
linear_score = 1.0
pca_ratio = 0


def on_connect(client, userdata, flags, rc):
    print("Connected with result code", str(rc))





def outlier_points_callback(data):
    global outlier_points_data
    # PointCloud2 데이터를 numpy array로 변환
    gen = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
    outlier_points_data = np.array(list(gen))

def outlier_depth_cloud_callback(data):
    global outlier_depth_cloud_data
    # PointCloud2 데이터를 numpy array로 변환
    gen = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
    outlier_depth_cloud_data = np.array(list(gen))

def linear_score_callback(data):
    global linear_score
    linear_score = data.data

def pca_ratio_callback(data):
    global pca_ratio
    pca_ratio = data.data

def listener():

    client = mqtt.Client()
    client.on_connect = on_connect

    client.connect("192.168.231.25", 1883, 60)

    client.loop_start()
    
    rospy.init_node('black_ice_node', anonymous=True)
    
    rospy.Subscriber("outlier_points", PointCloud2, outlier_points_callback)
    rospy.Subscriber("outlier_depth_cloud", PointCloud2, outlier_depth_cloud_callback)
    rospy.Subscriber("Linear_score", Float32, linear_score_callback)
    rospy.Subscriber("PCA_ratio", Float32, pca_ratio_callback)
    
    rate = rospy.Rate(5)

    global linear_score, pca_ratio, outlier_depth_cloud_data, outlier_points_data

    while not rospy.is_shutdown():
        black_ice_flag = "False"
        linear_flag = pca_flag = depth_cloud_flag = lidar_cloud_flag = 0
        
        if linear_score <0.96:
            linear_flag = 1
        if pca_ratio > 0.03:
            pca_flag = 3
        if len(outlier_depth_cloud_data) > 50:
            depth_cloud_flag = 1
        if len(outlier_points_data) > 5:
            lidar_cloud_flag = 1
        

        if linear_flag + pca_flag + depth_cloud_flag + lidar_cloud_flag >= 2:
            black_ice_flag = "True"
        client.publish("black_ice_data", black_ice_flag)
        print(linear_score, pca_ratio, len(outlier_depth_cloud_data), len(outlier_points_data))
        print("black_ice_data: ",black_ice_flag)
        rate.sleep()

if __name__ == '__main__':
    listener()

