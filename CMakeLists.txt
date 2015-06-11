cmake_minimum_required(VERSION 2.8.3)
project(pcl_segmentation)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  pcl_ros
  roscpp
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcl_segmentation
#  CATKIN_DEPENDS cv_bridge geometry_msgs pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pcl_node
  src/pcl_node.cpp)
target_link_libraries(pcl_node 
  ${catkin_LIBRARIES})
