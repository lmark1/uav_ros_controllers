/*
 * PlaneDetectionNode.cpp
 *
 *  Created on: Apr 8, 2019
 *      Author: lmark
 */

#include <ros/ros.h>

#include "PlaneDetection.h"

/**
 * Initializes plane detection node. Feeds PointCloud messages to
 * PlaneDetection object.
 *
 * Default topics for remapping:
 * 		- /pointcloud - PointCloud2 ROS message
 */
int main(int argc, char **argv) {

	ros::init(argc, argv, "plane_detection");
	ros::NodeHandle n;

	// Change logging level
	if (ros::console::set_logger_level(
		ROSCONSOLE_DEFAULT_NAME,
		ros::console::levels::Debug))
		ros::console::notifyLoggerLevelsChanged();

	// Make a new PlaneDetection object
	std::shared_ptr<PlaneDetection> planeDetection {new PlaneDetection()};
	ros::Subscriber rc_sub = n.subscribe("/pointcloud", 1,
			&PlaneDetection::pointCloudCallback, planeDetection.get());

	while(ros::ok())
		ros::spin();
}
