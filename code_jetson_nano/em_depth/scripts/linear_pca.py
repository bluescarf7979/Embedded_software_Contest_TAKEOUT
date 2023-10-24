#!/usr/bin/env python
# -*- coding: utf-8 -*-
import pcl  
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sklearn.decomposition import PCA
from sklearn.linear_model import LinearRegression
import numpy as np
import ros_numpy
from sklearn.linear_model import RANSACRegressor
from sklearn.neighbors import NearestNeighbors
from std_msgs.msg import Float32

def point_cloud_callback(msg):
    # Point cloud data to NumPy array
    pc_arr = ros_numpy.numpify(msg)
    points = np.zeros((pc_arr.shape[0], 3))
    
    points[:,0] = pc_arr['x']
    points[:,1] = pc_arr['y']
    points[:,2] = pc_arr['z']

    # Define the ROI (adjust the values as necessary)
    x_roi = (-4, -0.1)
    y_roi = (-0.3, 0.3)
    
    # Apply the ROI filter
    roi_mask = (points[:, 0] > x_roi[0]) & (points[:, 0] < x_roi[1]) & (points[:, 1] > y_roi[0]) & (points[:, 1] < y_roi[1])
    roi_points = points[roi_mask]

    # Check if there are any points left after filtering
    if roi_points.shape[0] == 0:
        print("No points remain after applying the ROI filter.")
        return
    
    
    # 각 포인트에 대한 k-NN을 사용하여 로컬 특징 계산
    k = 10  # 조절 가능한 파라미터
    nbrs = NearestNeighbors(n_neighbors=k).fit(points[:, :2])
    distances, indices = nbrs.kneighbors(roi_points[:, :2])
    
    # 각 포인트의 로컬 네이버후드에서 평균 거리 계산
    mean_distances = np.mean(distances, axis=1)
    
    # 평균 거리의 분포 분석
    global_mean = np.mean(mean_distances)
    global_std_dev = np.std(mean_distances)
    
    # 중심 극한 정리를 사용하여 이상치 식별 (2시그마 기준)
    outlier_mask = np.abs(mean_distances - global_mean) > 2 * global_std_dev
    
    # 이상치를 출력
    print("Number of outliers found:", np.sum(outlier_mask))
    
    # 이상치의 좌표 출력 (확인을 위함)
    outlier_points = roi_points[outlier_mask]
    print("Outlier points: ", len(outlier_points))

    cloud = pcl.PointCloud()
    cloud.from_array(roi_points.astype(np.float32))

    # Apply Voxel Grid filter for down-sampling the point cloud
    vox = cloud.make_voxel_grid_filter()
    vox.set_leaf_size(0.02, 0.02, 0.1)  # Adjust leaf size as needed
    cloud_filtered = vox.filter()
    points_filtered = cloud_filtered.to_array()
    
    X = points_filtered[:, 0].reshape(-1, 1) # x 좌표
    y = points_filtered[:, 1] # y 좌표

    # RANSAC 회귀 모델 적합
    ransac = RANSACRegressor()
    ransac.fit(X, y)

    # 예측 및 잔차 계산
    y_pred = ransac.predict(X)
    residuals = y - y_pred

    # 잔차의 표준 편차 계산
    std_dev = np.std(residuals)

    # 2시그마 기준으로 outlier 찾기
    outlier_mask = np.abs(residuals) > 2 * std_dev

    # 결과 출력
    print("Number of outliers found:",std_dev, np.sum(outlier_mask))
        

    # Create a new PointCloud2 message
    header = msg.header  # Reuse the header from the input point cloud message
    point_cloud2_msg = pc2.create_cloud_xyz32(header, points_filtered[:, :3])

    # Publish the PointCloud2 message
    pub.publish(point_cloud2_msg)


    # PCA Analysis
    pca = PCA(n_components=2)
    pca.fit(points_filtered[:, :2])
    print("PCA explained variance ratio: ", pca.explained_variance_ratio_)
    

    # Linear Regression
    lr = LinearRegression()
    lr.fit(points_filtered[:,0].reshape(-1,1), points_filtered[:,1])
    score = lr.score(points_filtered[:,0].reshape(-1,1), points_filtered[:,1])
    print("Linear regression score: ", score)
    print("")

    pub_pca.publish(std_dev)
    pub_lrs.publish(score)

rospy.init_node('point_cloud_analysis')
pub = rospy.Publisher('filtered_point_cloud', PointCloud2, queue_size=10)
pub_lrs = rospy.Publisher('/Linear_score', Float32, queue_size=10)
pub_pca = rospy.Publisher('/PCA_ratio', Float32, queue_size=10)

sub = rospy.Subscriber('/cloud', PointCloud2, point_cloud_callback)
rospy.spin()
