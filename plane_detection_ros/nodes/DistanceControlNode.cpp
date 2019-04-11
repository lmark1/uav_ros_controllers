/*
 * DistanceControlNode.cpp
 *
 *  Created on: Apr 11, 2019
 *      Author: lmark
 */

// ROS Includes
#include <ros/ros.h>
#include "DistanceControl.h"

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

	// Initialize distance control object
	std::shared_ptr<DistanceControl> distanceControl {new DistanceControl};

	// Setup callbacks
	ros::Subscriber distSub = nh.subscribe("/distance", 1,
			&DistanceControl::distanceCallback, distanceControl.get());
}





