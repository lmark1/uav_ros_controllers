/*
 * DistanceControlNode.cpp
 *
 *  Created on: Apr 11, 2019
 *      Author: lmark
 */

#include "control/DistanceControl.h"

// ROS Includes
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int32.h>

/**
 * Initializes distance control node.
 *
 * Default topics for remapping:
 * 		- /distance		- Distance from the UAV to the plane surface
 * 		- /joy			- Joystick topic used for enabling inspection mode
 *		- /real/imu		- IMU topic - realistic
 *		- /sim/odometry	- Odometry topic - simulation
 *
 * ROS parameters:
 * 		- simulation 	- Enable simulation mode
 * 		- rate			- Distance control rate
 */
int main(int argc, char **argv) {

	// Setup the node
	ros::init(argc, argv, "plane_detection");
	ros::NodeHandle nh;

	// Get mode parameter
	bool simMode = false;
	double rate = 25;
	nh.getParam("simulation", simMode);
	nh.getParam("rate", rate);

	// Initialize distance control object
	std::shared_ptr<DistanceControl> distanceControl
		{new DistanceControl (((simMode) ?
				DistanceControlMode::SIMULATION:
				DistanceControlMode::REALISTIC)) };

	// Setup callbacks
	ros::Subscriber distSub = nh.subscribe("/distance", 1,
			&ControlBase::distanceCb,
			dynamic_cast<ControlBase*>(distanceControl.get()));
	ros::Subscriber joySub = nh.subscribe("/joy", 1,
			&ControlBase::joyCb,
			dynamic_cast<ControlBase*>(distanceControl.get()));

	// Realistic callbacks
	ros::Subscriber imuSub = nh.subscribe("/real/imu", 1,
			&ControlBase::imuCbReal,
			dynamic_cast<ControlBase*>(distanceControl.get()));

	// Simulation callbacks
	ros::Subscriber odomSub = nh.subscribe("/sim/odometry", 1,
			&ControlBase::imuCbSim,
			dynamic_cast<ControlBase*>(distanceControl.get()));

	// Define publishers
	ros::Publisher statePub = nh.advertise<std_msgs::Int32>(
			"/control_state", 1);

	ros::Rate loopRate(rate);
	while (ros::ok())
	{
		ros::spinOnce();
		distanceControl->detectStateChange();
		distanceControl->publishState(statePub);
		// distanceControl->calculateSetpoint()

		/**
		 * If SIM
		 * 		distanceControl->publishSimSetpoint
		 * else
		 * 		distanceControl->publishRealSetpoint
		 */
		loopRate.sleep();
	}
}





