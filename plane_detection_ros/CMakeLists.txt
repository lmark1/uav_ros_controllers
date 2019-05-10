cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)
project(plane_detection_ros)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    dynamic_reconfigure
    sensor_msgs
    uav_ros_control
    mav_msgs)

find_package(PCL 1.8 REQUIRED COMPONENTS
    common
    io
    ros
    segmentation
    filters)

find_package(cmake_modules REQUIRED)


generate_dynamic_reconfigure_options(
    config/detection/PlaneDetectionParameters.cfg
    config/control/PositionControlParameters.cfg
    config/control/DistanceControlParameters.cfg
)

# TODO: Write the catkin_package command properly
catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS})
link_directories(
    ${PCL_LIBRARY_DIRS})
add_definitions(
    ${PCL_DEFINITIONS})

add_library(plane_detection 
    src/detection/DetectionBase.cpp
    src/detection/PlaneDetection.cpp
    src/detection/KalmanDetection.cpp)
target_link_libraries(plane_detection ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(distance_control 
    src/control/ControlBase.cpp
    src/control/DistanceControl.cpp
    src/control/JoyControl.cpp
    src/control/CarrotControl.cpp)
target_link_libraries(distance_control ${catkin_LIBRARIES})
add_dependencies(distance_control ${PROJECT_NAME}_gencfg)

add_executable(sequence_node 
    nodes/SequenceControlNode.cpp
    src/control/SequenceControl.cpp)   
target_link_libraries(sequence_node
    ${catkin_LIBRARIES})   

add_executable(plane_detection_node nodes/PlaneDetectionNode.cpp)
target_link_libraries(plane_detection_node 
    ${catkin_LIBRARIES} plane_detection)    
    
add_executable(distance_control_node nodes/DistanceControlNode.cpp)
target_link_libraries(distance_control_node
    ${catkin_LIBRARIES} distance_control)

add_executable(carrot_control_node nodes/CarrotControlNode.cpp)
target_link_libraries(carrot_control_node
    ${catkin_LIBRARIES} distance_control)