#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from sklearn.decomposition import PCA
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error

def callback(data):
    pc_arr = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
    points = np.array(list(pc_arr))

    # 각도로 필터링하기 위해 points를 극 좌표계로 변환
    angles = np.arctan2(points[:,1], points[:,0])
    
    # 특정 각도 범위 (예: -45° to 45°) 내의 포인트만 선택
    x_range = (-3.0, -0.1)
    y_range = (-0.3, 0.3)
    angle_range = (-np.pi, np.pi)
    
    roi_indices = np.where((points[:,0] >= x_range[0]) & (points[:,0] <= x_range[1]) & 
                           (points[:,1] >= y_range[0]) & (points[:,1] <= y_range[1]) &
                           (angles >= angle_range[0]) & (angles <= angle_range[1]))

    points = points[roi_indices]

    # # PointCloud2 메시지를 numpy array로 변환
    # pc = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
    # points = np.array(list(pc))
    
    # 데이터가 충분하지 않은 경우 처리를 건너뜁니다
    if len(points) < 2:
        return

    # PCA
    pca = PCA(n_components=2)
    principalComponents = pca.fit_transform(points[:, :2])

    # 선형 회귀
    x = points[:, 0].reshape(-1, 1)
    y = points[:, 1].reshape(-1, 1)
    lr = LinearRegression()
    lr.fit(x, y)
    predictions = lr.predict(x)

    # 선형성 평가 (mean squared error를 사용)
    linearity_score = mean_squared_error(y, predictions)

    # 이상치 식별 (정상 범위를 벗어난 예측 오차를 가진 포인트 찾기)
    threshold = 0.14  # 이상치 판단 기준이 될 임계값; 이 값을 조정할 수 있습니다
    errors = np.abs(y - predictions)
    outlier_points = points[np.squeeze(errors) > threshold]

    # 결과 출력
    
    rospy.loginfo("선형성 점수: %f", linearity_score)
    rospy.loginfo( len(points))
    rospy.loginfo("이상치 개수: %d", len(outlier_points))
    # rospy.loginfo("이상치 위치:\n%s", outliers)
    header = data.header
    outlier_msg = pc2.create_cloud_xyz32(header, outlier_points)

    # 이상치 포인트들을 발행
    outlier_pub.publish(outlier_msg)
    rospy.loginfo('Published %s outlier points', len(outlier_points))

def listener():
    rospy.init_node('pointcloud_processor', anonymous=True)
    rospy.Subscriber("/cloud", PointCloud2, callback)
    global outlier_pub
    outlier_pub = rospy.Publisher("/outlier_points", PointCloud2, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    listener()
