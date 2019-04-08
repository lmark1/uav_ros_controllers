/*
 * PlaneDetectionNode.cpp
 *
 *  Created on: Apr 8, 2019
 *      Author: lmark
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "PlaneDetection.h"

/**
 * Initializes plane detection node. Feeds PointCloud messages to
 * PlaneDetection object.
 */
int main(int argc, char **argv) {

	// Initialize ROS node.
	ros::init(argc, argv, "plane_detection");

	PlaneDetection planeDetection;
}
