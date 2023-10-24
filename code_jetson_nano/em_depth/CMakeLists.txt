cmake_minimum_required(VERSION 3.0.2)

add_compile_options(-std=c++14)

project(em_depth)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  cv_bridge
  image_geometry
  laser_geometry
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
    INCLUDE_DIRS include
#  LIBRARIES targetless
  CATKIN_DEPENDS cv_bridge image_geometry pcl_ros roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${realsense2_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE project_HEADERS
    "*.h"
    "*.hpp"
)

add_executable(
${PROJECT_NAME}
${project_HEADERS}
src/em_depth_node.cpp 
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${realsense2_LIBRARY}
)


