cmake_minimum_required(VERSION 2.8.3)
project(delphi_husong)

set(PCL_DIR "/home/husong/Downloads/pcl-pcl-1.8.1/build/")
find_package(PCL 1.8.1 EXACT REQUIRED COMPONENTS common io sample_consensus visualization registration filters keypoints)
include_directories(${PCL_INCLUDE_DIRS} /home/husong/Downloads/pcl-pcl-1.8.1/build/include include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  pcl_conversions
  velodyne_pointcloud
)

catkin_package(CATKIN_DEPENDS
	sensor_msgs
	std_msgs
	velodyne_pointcloud
)


#set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.8/)  #指定pcl1.8路径



message(STATUS
	${PCL_LIBRARY_DIRS}
	${PCL_DEFINITIONS})

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
)


SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(delphi_husong
	src/main.cpp
)

target_include_directories(delphi_husong PRIVATE
	${PCL_INCLUDE_DIRS}
)

target_link_libraries(delphi_husong
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)