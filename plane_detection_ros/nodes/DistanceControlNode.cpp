/*
 * DistanceControlNode.cpp
 *
 *  Created on: Apr 11, 2019
 *      Author: lmark
 */

#include <plane_detection_ros/control/DistanceControl.h>
#include <plane_detection_ros/DistanceControlParametersConfig.h>

// ROS Includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <std_msgs/Int32.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <std_msgs/Bool.h>

/**
 * Initializes distance control node.
 *
 * Default topics for remapping:
 * 		- /distance		- Distance from the UAV to the plane surface
 * 		- /joy			- Joystick topic used for enabling inspection mode
 *		- /real/imu		- IMU topic - realistic
 *		- /real/pos		- Local position topic - realistic
 *		- /real/vel		- Velocity topic - realistic
 *		- /sim/odometry	- Odometry topic - simulation
 */
int main(int argc, char **argv) 
{
	// Setup the node
	ros::init(argc, argv, "distance_control");
	ros::NodeHandle nh;
	// Change logging level
	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
		ros::console::levels::Debug))
		ros::console::notifyLoggerLevelsChanged();

	// Check if sim mode or real
	bool simMode = false;
	bool initialized = nh.getParam("/control/sim_mode", simMode);
	dist_control::DistanceControlMode mode = 
		((simMode) ? 
			dist_control::DistanceControlMode::SIMULATION : 
			dist_control::DistanceControlMode::REAL);
	// Initialize distance control object
	std::shared_ptr<dist_control::DistanceControl> distanceControl
		{ new dist_control::DistanceControl(mode, nh) };

	// Run default distance control
	dist_control::runDefault(*distanceControl, nh, simMode);	
}





