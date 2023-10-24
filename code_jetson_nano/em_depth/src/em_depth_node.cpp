#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <cmath>
#include <vector>
#include <numeric>
#include <ros/ros.h>
#include <std_msgs/Float64.h>


class KalmanFilter {
private:
    double process_variance;
    double measurement_variance;
    double estimate;
    double estimate_error;

public:
    KalmanFilter(double process_var, double measurement_var, double initial_value = 0, double initial_estimate_error = 1) 
        : process_variance(process_var), measurement_variance(measurement_var), estimate(initial_value), estimate_error(initial_estimate_error) {}

    void update(double measurement) {
        double prediction = estimate;
        double prediction_error = estimate_error + process_variance;
        double kalman_gain = prediction_error / (prediction_error + measurement_variance);
        estimate = prediction + kalman_gain * (measurement - prediction);
        estimate_error = (1 - kalman_gain) * prediction_error;
    }

    double getEstimate() {
        return estimate;
    }
};

std::pair<double, double> calculate_angle_and_distance(const rs2::vertex& p1, double x2, double y2) {
    double vertical_distance = p1.z;
    double distance = std::sqrt(std::pow(x2 - p1.x, 2) + std::pow(y2 - p1.y, 2) + std::pow(p1.z, 2));
    double angle_rad = std::atan2(p1.z, std::sqrt(std::pow(x2 - p1.x, 2) + std::pow(y2 - p1.y, 2)));
    double angle_deg = (angle_rad * 180.0) / M_PI;
    return {angle_deg, distance};
}

rs2::vertex rotate_point_around_x(const rs2::vertex& p, double angle_deg) {
    double angle_rad = (angle_deg * M_PI) / 180.0;
    return {
        p.x,
        p.y * std::cos(angle_rad) - p.z * std::sin(angle_rad),
        p.y * std::sin(angle_rad) + p.z * std::cos(angle_rad)
    };
}

class VoxelFilter {
public:
   VoxelFilter() {
      ros::NodeHandle nh_;
      // sub_ = nh_.subscribe("/camera/depth/color/points", 1, &VoxelFilter::pointCloudCallback, this);
      pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/outlier_depth_cloud", 1);
      pubground = nh_.advertise<sensor_msgs::PointCloud2>("/ground", 1);
      nh_.getParam(ros::this_node::getName() + "/floor_distance", floor_distace);
      nh_.getParam(ros::this_node::getName()+"/hsv_lower", hsv_lower);
      nh_.getParam(ros::this_node::getName()+"/hsv_upper", hsv_upper);
      if (!nh_.getParam(ros::this_node::getName() + "/voxel_size", voxel_size_)) {
         ROS_WARN("Voxel size parameter not set, using default value: 0.1");
         voxel_size_ = 0.1;
      }
      minH = hsv_lower[0];
      minS = hsv_lower[1];
      minV = hsv_lower[2];
      maxH = hsv_upper[0];
      maxS = hsv_upper[1];
      maxV = hsv_upper[2];
      this->count=0;
   }
   pcl::ModelCoefficients::Ptr segmentGroundPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointIndices::Ptr& inliers)
   {
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      pcl::SACSegmentation<pcl::PointXYZRGB> seg;
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(0.01);
      seg.setInputCloud(cloud);
      seg.segment(*inliers, *coefficients);

      return coefficients;
   }
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointsAboveGround(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const pcl::ModelCoefficients::Ptr& coefficients, double threshold)
   {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
      for (size_t i = 0; i < cloud->points.size(); ++i)
      {
         pcl::PointXYZRGB point = cloud->points[i];
         double distance_to_plane = coefficients->values[0] * point.x + coefficients->values[1] * point.y + coefficients->values[2] * point.z + coefficients->values[3];
         // if(distance_to_plane < 0)
         // {
         //    ROS_INFO("Voxel size parameter not set, using default value: 0.1");
         // }
         distance_to_plane = std::abs(distance_to_plane);
         if (distance_to_plane > threshold)
         {
               cloud_filtered->points.push_back(point);
         }
      }

      return cloud_filtered;
   }
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterByHSV(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud)
   {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

      for (const auto& point : input_cloud->points)
      {
         cv::Mat bgr(1, 1, CV_8UC3, cv::Scalar(point.b, point.g, point.r));
         cv::Mat hsv;

         cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

         int h = hsv.at<cv::Vec3b>(0, 0)[0];
         int s = hsv.at<cv::Vec3b>(0, 0)[1];
         int v = hsv.at<cv::Vec3b>(0, 0)[2];

         if (h >= minH && h <= maxH && s >= minS && s <= maxS && v >= minV && v <= maxV)
         {
               filtered_cloud->points.push_back(point);
         }
      }

      filtered_cloud->width = filtered_cloud->points.size();
      filtered_cloud->height = 1;
      filtered_cloud->is_dense = true;

      return filtered_cloud;
   }
   int count;
   int minH, maxH, minS, maxS, minV, maxV;
   std::vector<int> hsv_lower, hsv_upper;

   void getPointCloudFromSDK(rs2::pipeline pipe) {
      this->count++;
      if(this->count%3 != 0) return;

      rs2::pointcloud pc;
      rs2::points points;

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

      auto frames = pipe.wait_for_frames();

      // Fetch the depth frame
      auto depth = frames.get_depth_frame();

      // Generate the point cloud
      points = pc.calculate(depth);

      // Get the vertices (XYZ coordinates) of the point cloud
      auto vertices = points.get_vertices();

      for (int i = 0; i < points.size(); i+=10) {
         // You can access the X, Y, Z coordinates like this:
         float x = vertices[i].x;
         float y = vertices[i].y;
         float z = vertices[i].z;

         pcl::PointXYZRGB pt;
         pt.x = x;
         pt.y = y;
         pt.z = z;
         cloud->points.push_back(pt);
      }
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_above_ground;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr hsv_cloud;

      pcl::VoxelGrid<pcl::PointXYZRGB> sor;
      sor.setInputCloud(cloud);
      sor.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
      sor.filter(*cloud_filtered);

      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      pcl::ModelCoefficients::Ptr coefficients = segmentGroundPlane(cloud_filtered, inliers);
      
      cloud_above_ground = getPointsAboveGround(cloud_filtered, coefficients, floor_distace);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setInputCloud(cloud_filtered);
      extract.setIndices(inliers);
      extract.setNegative(false);
      extract.filter(*ground_cloud);

      hsv_cloud = filterByHSV(cloud_above_ground);
      ROS_WARN("hsv_cloud size %ld", hsv_cloud->points.size());

      sensor_msgs::PointCloud2 output;
      pcl::toROSMsg(*cloud_above_ground, output);
      // std::cout<<cloud_msg->header.frame_id<<"\n";
      output.header.frame_id = "velodyne";
      pub_.publish(output);

      sensor_msgs::PointCloud2 ground_msg;
      pcl::toROSMsg(*ground_cloud, ground_msg);
      ground_msg.header.frame_id = "velodyne";
      pubground.publish(ground_msg);
   }
private:

   ros::Subscriber sub_;
   ros::Publisher pub_, pubground;
   double voxel_size_, floor_distace;
};

int main(int argc, char** argv) {
   ros::init(argc, argv, "em_depth_node");
   ros::NodeHandle nh_;
   ros::Publisher estimate_pub = nh_.advertise<std_msgs::Float64>("estimated_depth", 10);

   VoxelFilter voxel_filter;

   rs2::pipeline pipe;
   pipe.start();

   KalmanFilter kf(1, 10);
   const double RADIUS = 0.3;

   ros::Rate r(5);
   while (ros::ok()) {
      voxel_filter.getPointCloudFromSDK(pipe);

      rs2::frameset frames = pipe.wait_for_frames();
      rs2::depth_frame depth_frame = frames.get_depth_frame();
      const int width = depth_frame.get_width();
      const int height = depth_frame.get_height();
      const int center_x = width / 2;
      const int center_y = height / 2;
      std::vector<double> roi_values;

      for (int y = 0; y < height; y+=5) {
         for (int x = 0; x < width; x+=5) {
               rs2::vertex pt = {static_cast<float>(x), static_cast<float>(y), depth_frame.get_distance(x, y)};
               auto [angle, distance] = calculate_angle_and_distance(pt, center_x, center_y);
               rs2::vertex rotated_p = rotate_point_around_x(pt, -angle);
               double dist_from_center = std::sqrt(std::pow(rotated_p.x - center_x, 2) + std::pow(rotated_p.y - center_y, 2));
               if (dist_from_center < (RADIUS * 1000)) {
                  roi_values.push_back(rotated_p.z);
               }
         }
      }

      double mean_depth_in_roi = std::accumulate(roi_values.begin(), roi_values.end(), 0.0) / roi_values.size();
      kf.update(mean_depth_in_roi);

      std::cout << "Filtered Mean Depth in ROI: " << kf.getEstimate() / 1000 << std::endl;
      std_msgs::Float64 estimate_msg;
      estimate_msg.data = kf.getEstimate() / 1000;
      estimate_pub.publish(estimate_msg);
      r.sleep();
      ros::spinOnce();   
   }
   
   return 0;
}
