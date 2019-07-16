cmake_minimum_required(VERSION 2.8.3)
project(uav_ros_control)

find_package(catkin REQUIRED COMPONENTS
	cmake_modules
	roscpp
	uav_ros_control_msgs
	rospy
    std_msgs
    dynamic_reconfigure
    sensor_msgs
    mav_msgs)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES pid nfilt kalman
	CATKIN_DEPENDS uav_ros_control_msgs)

include_directories(
	include
	${catkin_INCLUDE_DIRS})

# PID controller library
add_library(pid src/control/PID.cpp)
target_link_libraries(pid ${catkin_LIBRARIES})

# Nonlinear filters library
add_library(nfilt src/filters/NonlinearFilters.cpp)
target_link_libraries(nfilt ${catkin_LIBRARIES})

# Kalman filter library
add_library(kalman src/filters/KalmanFilter.cpp)
target_link_libraries(kalman ${catkin_LIBRARIES})

# Carrot reference generation node
add_executable(carrot_reference_node
	src/reference/CarrotReference.cpp
	src/reference/JoyControlInput.cpp
	nodes/reference/CarrotReferenceNode.cpp)
target_link_libraries(carrot_reference_node 
	${catkin_LIBRARIES} nfilt)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
