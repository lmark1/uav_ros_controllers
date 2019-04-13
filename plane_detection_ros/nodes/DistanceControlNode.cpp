/*
 * DistanceControlNode.cpp
 *
 *  Created on: Apr 11, 2019
 *      Author: lmark
 */

// ROS Includes
#include <ros/ros.h>
#include "control/DistanceControl.h"

/**
 * Initializes distance control node.
 *
 * Default topics for remapping:
 * 		- /distance		- Distance from the UAV to the plane surface
 *
 */
int main(int argc, char **argv) {

	// Setup the node
	ros::init(argc, argv, "plane_detection");
	ros::NodeHandle nh;

	// Get mode parameter
	bool simMode = false;
	nh.getParam("simulation", simMode);
	DistanceControlMode mode = ((simMode) ?
			DistanceControlMode::SIMULATION:
			DistanceControlMode::REALISTIC);

	// Initialize distance control object
	std::shared_ptr<DistanceControl> distanceControl
		{new DistanceControl(mode) };

	// Setup callbacks
	ros::Subscriber distSub = nh.subscribe("/distance", 1,
			&ControlBase::distanceCb,
			dynamic_cast<ControlBase*>(distanceControl.get()));
	ros::Subscriber joySub = nh.subscribe("/joy", 1,
			&ControlBase::joyCb,
			dynamic_cast<ControlBase*>(distanceControl.get()));

	// Realistic callbacks
	ros::Subscriber imuSub = nh.subscribe("/imu", 1,
			&ControlBase::imuCbReal,
			dynamic_cast<ControlBase*>(distanceControl.get()));

	// Simulation callbacks
	ros::Subscriber odomSub = nh.subscribe("/odometry", 1,
			&ControlBase::imuCbSim,
			dynamic_cast<ControlBase*>(distanceControl.get()));
}





