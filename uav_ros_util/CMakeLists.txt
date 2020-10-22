cmake_minimum_required(VERSION 3.10.0)
project(uav_ros_util)

find_package(catkin REQUIRED COMPONENTS roscpp dynamic_reconfigure mavros)

catkin_package(INCLUDE_DIRS include LIBRARIES CATKIN_DEPENDS)

include_directories(include ${catkin_INCLUDE_DIRS})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")