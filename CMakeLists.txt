cmake_minimum_required(VERSION 2.8.3)
project(pcl_segmentation)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  pcl_ros
  roscpp
  sensor_msgs
  image_geometry
  message_generation
)

add_message_files(
  FILES
  ObjectClusters.msg	
  ObjectCluster.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pcl_node
  src/pcl_node.cpp)
target_link_libraries(pcl_node 
  ${catkin_LIBRARIES})
