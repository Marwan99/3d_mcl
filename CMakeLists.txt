cmake_minimum_required(VERSION 3.1)
project(mcl)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  tf2
  tf2_ros
  tf2_msgs
  roscpp
  # pcl library
  pcl_conversions
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  rosbag
  std_srvs
)

find_package(PCL REQUIRED QUIET COMPONENTS common kdtree filters)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL

  # CATKIN_DEPENDS
  # std_msgs
  # nav_msgs
  # geometry_msgs
  # sensor_msgs
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
)

###########
## Build ##
###########

add_library(motion_model
  src/motion_model.cpp
)

target_link_libraries(motion_model
  ${catkin_LIBRARIES}
  gtsam
)

add_library(measurment_model
  src/measurment_model.cpp
)

target_link_libraries(measurment_model
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(mcl
  src/mcl.cpp
)

target_link_libraries(mcl
  ${catkin_LIBRARIES}
  measurment_model
  motion_model
)

# node excutable
add_executable(${PROJECT_NAME}_node
  src/mcl_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  mcl
)
