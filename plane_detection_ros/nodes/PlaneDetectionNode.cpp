/*
 * PlaneDetectionNode.cpp
 *
 *  Created on: Apr 8, 2019
 *      Author: lmark
 */

// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include "plane_detection_ros/detection/DetectionWrapper.h"

/**
 * Initializes plane detection node. Feeds PointCloud messages to
 * PlaneDetection object.
 *
 * Default topics for remapping:
 * 		- /pointcloud 	- INPUT  - PointCloud2 ROS message
 * 		- /plane	  	- OUTPUT - PointCloud2 ROS message representing a plane
 * 		- /plane_normal - OUTPUT -PoseStamped message representing the plane
 * 					      centroid an normal orientation
 *		- /distance		- OUTPUT -Distance from the UAV to the plane
 */

int main(int argc, char **argv) {

	// Setup the node
	ros::init(argc, argv, "plane_detection");
	ros::NodeHandle nh;
	
	// Read all available parameters
	std::string frameID {"velodyne"};
	double rate = 10;
	double kalmanNoiseMv = 10;
	double kalmanNoisePos = 1;
	double kalmanNoiseVel = 10;
	nh.getParam("/detection/frame_id", frameID);
	nh.getParam("/detection/rate", rate);
	nh.getParam("/kalman/noise_mv", kalmanNoiseMv);
	nh.getParam("/kalman/noise_pos", kalmanNoisePos);
	nh.getParam("/kalman/noise_vel", kalmanNoiseVel);

	// Change logging level
	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
		ros::console::levels::Info))
		ros::console::notifyLoggerLevelsChanged();

	// Setup a new planeDetection object
	std::shared_ptr<DetectionWrapper> detectionWrapper
		{new DetectionWrapper (frameID, kalmanNoiseMv, kalmanNoisePos, kalmanNoiseVel)};
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
	// Set initial reconfigure parameters
	detectionWrapper->setReconfigureParameters(confServer);

	// Setup reconfigure server
	paramCallback = boost::bind(
			&DetectionWrapper::parametersCallback,
			detectionWrapper, _1, _2);
	confServer.setCallback(paramCallback);

	// Define some publishers
	ros::Publisher planePub = nh.advertise<sensor_msgs::PointCloud2>(
			"/plane", 1);
	ros::Publisher normalPub = nh.advertise<geometry_msgs::PoseStamped>(
			"/plane_normal", 1);
	ros::Publisher distPub = nh.advertise<std_msgs::Float64>(
			"/distance", 1);
	ros::Publisher distFiltPub = nh.advertise<std_msgs::Float64>(
			"/distance_filtered", 1);

	// Setup the loop
	ROS_INFO("Setting rate to %.2f", rate);
	ros::Rate loopRate {rate};
	double dt = 1.0 / rate;
	while(ros::ok())
	{
		ros::spinOnce();
		detectionWrapper->convertFromROSMsg();

		// Detect normal
		detectionWrapper->doCloudFiltering();
		detectionWrapper->doPlaneDetection();
		detectionWrapper->publishPlane(planePub);
		detectionWrapper->publishNormal(normalPub);

		// Calculate distance
		detectionWrapper->calculateDistanceToPlane();
		detectionWrapper->publishDistanceToPlane(distPub);

		// Filter distance
		detectionWrapper->filterCurrentDistance(dt);
		detectionWrapper->publishFilteredDistance(distFiltPub);

		loopRate.sleep();
	}

	return 0;
}
