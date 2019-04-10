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
 * 		- /pointcloud 	- PointCloud2 ROS message
 * 		- /plane	  	- PointCloud2 ROS message representing a plane
 * 		- /plane_normal - PoseStamped message representing the plane
 * 					      centroid an normal orientation
 *		- /distance		- Distance from the UAV to the plane
 */
int main(int argc, char **argv) {

	// Setup the node
	ros::init(argc, argv, "plane_detection");
	ros::NodeHandle nh;


	// Change logging level
	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
		ros::console::levels::Warn))
		ros::console::notifyLoggerLevelsChanged();

	// Setup a new planeDetection object
	std::shared_ptr<DetectionWrapper> detectionWrapper {new DetectionWrapper};
	ros::Subscriber pclSub = nh.subscribe("/pointcloud", 1,
			&DetectionWrapper::pointCloudCallback, detectionWrapper.get());

	// Initialize configure server
	dynamic_reconfigure::
		Server<plane_detection_ros::PlaneDetectionParametersConfig>
		confServer;

	// Initialize reconfigure callback
	dynamic_reconfigure::
		Server<plane_detection_ros::PlaneDetectionParametersConfig>::CallbackType
		paramCallback;

	// Setup reconfigure server
	paramCallback = boost::bind(
			&DetectionWrapper::parametersCallback,
			*detectionWrapper, _1, _2);
	confServer.setCallback(paramCallback);

	// Define some publishers
	ros::Publisher planePub = nh.advertise<sensor_msgs::PointCloud2>(
			"/plane", 1);
	ros::Publisher normalPub = nh.advertise<geometry_msgs::PoseStamped>(
			"/plane_normal", 1);
	ros::Publisher distPub = nh.advertise<std_msgs::Float64>(
			"/distance", 1);

	// Setup the loop
	ros::Rate loopRate {detectionWrapper->getDetectionRate()};
	while(ros::ok())
	{
		ros::spinOnce();
		detectionWrapper->convertFromROSMsg();
		detectionWrapper->doCloudFiltering();
		detectionWrapper->doPlaneDetection();
		detectionWrapper->publishPlane(planePub);
		detectionWrapper->publishNormal(normalPub);
		detectionWrapper->publishDistanceToPlane(distPub);
		loopRate.sleep();
	}

	return 0;
}
