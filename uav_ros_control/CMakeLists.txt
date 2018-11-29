cmake_minimum_required(VERSION 2.8.3)
project(uav_ros_control)

find_package(catkin REQUIRED COMPONENTS
	cmake_modules
	uav_ros_control_msgs)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES pid nfilt)

include_directories(
	include
	${catkin_INCLUDE_DIRS})

# PID controller library
add_library(pid src/PID.cpp)
target_link_libraries(pid ${catkin_LIBRARIES})

# Nonlinear filters library
add_library(nfilt src/NonlinearFilters.cpp)
target_link_libraries(nfilt ${catkin_LIBRARIES})