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
	ros::NodeHandle nh;

	// Change logging level
	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
		ros::console::levels::Debug))
		ros::console::notifyLoggerLevelsChanged();

	// Setup a new planeDetection object
	std::shared_ptr<PlaneDetection> planeDetection {new PlaneDetection};
	ros::Subscriber pclSub = nh.subscribe("/pointcloud", 1,
			&PlaneDetection::pointCloudCallback, planeDetection.get());
	ros::Publisher planePub = nh.advertise<sensor_msgs::PointCloud2>(
			"/plane", 1);

	// Setup the loop
	ros::Rate loopRate {planeDetection->getDetectionRate()};
	while(ros::ok())
	{
		ros::spinOnce();
		planeDetection->detectPlane();
		planeDetection->publishPlane(planePub);
		loopRate.sleep();
	}

	return 0;
}
