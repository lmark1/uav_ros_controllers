/*
 * PlaneDetectionNode.cpp
 *
 *  Created on: Apr 8, 2019
 *      Author: lmark
 */

// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include "DetectionWrapper.h"

/**
 * Initializes plane detection node. Feeds PointCloud messages to
 * PlaneDetection object.
 *
 * Default topics for remapping:
 * 		- /pointcloud - PointCloud2 ROS message
 */
int main(int argc, char **argv) {

	// Setup the node
	ros::init(argc, argv, "plane_detection");
	ros::NodeHandle nh;

	// Initialize configure server
	dynamic_reconfigure::
		Server<plane_detection_ros::PlaneDetectionParametersConfig>
		confServer;

	// Initialize reconfigure callback
	dynamic_reconfigure::
		Server<plane_detection_ros::PlaneDetectionParametersConfig>::CallbackType
		paramCallback;

	// Change logging level
	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
		ros::console::levels::Debug))
		ros::console::notifyLoggerLevelsChanged();

	// Setup a new planeDetection object
	std::shared_ptr<DetectionWrapper> detectionWrapper {new DetectionWrapper};
	ros::Subscriber pclSub = nh.subscribe("/pointcloud", 1,
			&DetectionWrapper::pointCloudCallback, detectionWrapper.get());

	// Setup reconfigure server
	paramCallback = boost::bind(
			&DetectionWrapper::parametersCallback,
			*detectionWrapper, _1, _2);
	confServer.setCallback(paramCallback);

	ros::Publisher planePub = nh.advertise<sensor_msgs::PointCloud2>(
			"/plane", 1);

	// Setup the loop
	ros::Rate loopRate {detectionWrapper->getDetectionRate()};
	while(ros::ok())
	{
		ros::spinOnce();
		detectionWrapper->convertFromROSMsg();
		detectionWrapper->doCloudFiltering();
		detectionWrapper->doPlaneDetection();
		detectionWrapper->publishPlane(planePub);
		loopRate.sleep();
	}

	return 0;
}
