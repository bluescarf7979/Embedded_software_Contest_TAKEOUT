#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

class LaserToPointcloudConverter {
private:
//변수 정의
  ros::NodeHandle nh_;
  ros::Subscriber scan_sub_;
  ros::Publisher cloud_pub_;
  laser_geometry::LaserProjection projector_;
int count;
public:
  LaserToPointcloudConverter() {
count=0;
//scan_sub 서브스크라이버는 scanCallback 함수를 scan 토픽에서 메시지를 수신할때마다 호출
    scan_sub_ = nh_.subscribe("scan", 1, &LaserToPointcloudConverter::scanCallback, this);
//cloud_pub 퍼블리셔는 cloud라는 이름의 PointCloud2 토픽 발행
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud", 1);
  }

//LaserScan 타입의 2D LiDAR 센서의 토픽 받아서 cloud라는 이름의 PointCloud2 데이터로 변환
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
count++;	
if(count%4 !=0) return;
    sensor_msgs::PointCloud2 cloud;
    projector_.projectLaser(*scan, cloud);
    cloud_pub_.publish(cloud);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "laser_to_pointcloud_node");
  LaserToPointcloudConverter converter;
  ros::spin();
  return 0;
}
