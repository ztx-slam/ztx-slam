cmake_minimum_required(VERSION 2.8.3)
project(ztx_slam)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  pcl_ros
  roscpp
  rospy
  std_msgs
  tf
  pcl_conversions)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs tf pcl_conversions
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS include
  LIBRARIES ztx
)

## Compile as C++14, supported in ROS Kinetic and newer
# set_property(TARGET invz_player PROPERTY CXX_STANDARD 17)
add_compile_options(-std=c++11)

#add_definitions( -march=native )


add_subdirectory(src/lib)
add_executable(preprocess_node src/preprocess_node.cpp)
target_link_libraries(preprocess_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ztx )

