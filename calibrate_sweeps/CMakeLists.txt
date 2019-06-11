cmake_minimum_required(VERSION 2.8.3)
project(calibrate_sweeps)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp message_generation qt_build semantic_map metaroom_xml_parser strands_sweep_registration actionlib actionlib_msgs)

set(CMAKE_CXX_FLAGS "-O4 -fPIC -std=c++0x -fpermissive ${CMAKE_CXX_FLAGS}")





set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl/")



find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



rosbuild_prepare_qt4(QtCore QtXml)

add_action_files(
DIRECTORY
    action
FILES
    CalibrateSweeps.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
)


catkin_package(
   INCLUDE_DIRS
   LIBRARIES 
   CATKIN_DEPENDS semantic_map metaroom_xml_parser strands_sweep_registration actionlib actionlib_msgs
   DEPENDS libceres-dev suitesparse qt_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
