cmake_minimum_required(VERSION 3.10.0)
project(uav_ros_tracker)

find_package(catkin REQUIRED COMPONENTS roscpp dynamic_reconfigure mavros)

catkin_package(INCLUDE_DIRS include LIBRARIES CATKIN_DEPENDS)

include_directories(include ${catkin_INCLUDE_DIRS})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

add_library(
  mpc_tracker_cvxgen
  src/cvx_wrapper.cpp src/cvxgen/ldl.c
  src/cvxgen/matrix_support.c src/cvxgen/solver.c
  src/cvxgen/util.c)
