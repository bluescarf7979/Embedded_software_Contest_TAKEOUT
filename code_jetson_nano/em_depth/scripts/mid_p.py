#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import pyrealsense2 as rs
import math

class KalmanFilter:
    def __init__(self, process_variance, measurement_variance, initial_value=0, initial_estimate_error=1):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = initial_value
        self.estimate_error = initial_estimate_error

    def update(self, measurement):
        prediction = self.estimate
        prediction_error = self.estimate_error + self.process_variance
        kalman_gain = prediction_error / (prediction_error + self.measurement_variance)
        self.estimate = prediction + kalman_gain * (measurement - prediction)
        self.estimate_error = (1 - kalman_gain) * prediction_error

    def getEstimate(self):
        return self.estimate

def calculate_angle_and_distance(p1, x2, y2):
    vertical_distance = p1[2]
    distance = math.sqrt((x2 - p1[0]) ** 2 + (y2 - p1[1]) ** 2 + p1[2] ** 2)
    angle_rad = math.atan2(p1[2], math.sqrt((x2 - p1[0]) ** 2 + (y2 - p1[1]) ** 2))
    angle_deg = math.degrees(angle_rad)
    return angle_deg, distance

def rotate_point_around_x(p, angle_deg):
    angle_rad = math.radians(angle_deg)
    rotation_matrix = np.array([
        [1, 0, 0],
        [0, math.cos(angle_rad), -math.sin(angle_rad)],
        [0, math.sin(angle_rad), math.cos(angle_rad)]
    ])
    rotated_point = np.dot(rotation_matrix, p)
    return rotated_point

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

kf = KalmanFilter(process_variance=1, measurement_variance=10)
RADIUS = 0.3

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        
        center_y, center_x = depth_image.shape[0] // 2, depth_image.shape[1] // 2
        roi_values = []

        for y in range(depth_image.shape[0]):
            for x in range(depth_image.shape[1]):
                pt = [x, y, depth_image[y, x]]
                angle, distance = calculate_angle_and_distance(pt, center_x, center_y)
                rotated_p = rotate_point_around_x(pt, -angle)
                dist_from_center = math.sqrt((rotated_p[0] - center_x) ** 2 + (rotated_p[1] - center_y) ** 2)
                if dist_from_center < (RADIUS * 1000):
                    roi_values.append(rotated_p[2])

        mean_depth_in_roi = np.mean(roi_values)
        kf.update(mean_depth_in_roi)

        print("Filtered Mean Depth in ROI:",kf.getEstimate() / 1000)
        
except KeyboardInterrupt:
    pass

finally:
    pipeline.stop()

