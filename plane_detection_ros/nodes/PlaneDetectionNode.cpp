/*
 * PlaneDetectionNode.cpp
 *
 *  Created on: Apr 8, 2019
 *      Author: lmark
 */

// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

// Own includes
#include "plane_detection_ros/detection/KalmanDetection.h"
#include <plane_detection_ros/PlaneDetectionParametersConfig.h>

// Cpp includes
#include <array>

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
	
	// Change logging level
	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
		ros::console::levels::Info))
		ros::console::notifyLoggerLevelsChanged();

	// Setup a new planeDetection object
	std::shared_ptr<KalmanDetection> planeDetect { new KalmanDetection };
	planeDetect->initializeParameters(nh);
	
	boost::recursive_mutex config_mutex;
	// Initialize configure server
	dynamic_reconfigure::
		Server<plane_detection_ros::PlaneDetectionParametersConfig>
		confServer {config_mutex};
	// Initialize reconfigure callback
	dynamic_reconfigure::
		Server<plane_detection_ros::PlaneDetectionParametersConfig>::CallbackType
		paramCallback;
	
	// Set initial reconfigure parameters
	plane_detection_ros::PlaneDetectionParametersConfig config;
	confServer.setConfigDefault(config);
	planeDetect->setReconfigureParameters(config);
	confServer.updateConfig(config);

	// Setup reconfigure server
	paramCallback = boost::bind(
			&PlaneDetection::parametersCallback,
			planeDetect, _1, _2);
	confServer.setCallback(paramCallback);

	// Pointcloud subscriber
	ros::Subscriber pclSub = nh.subscribe("/pointcloud", 1,
		&PlaneDetection::pointCloudCallback, 
		dynamic_cast<DetectionBase*>(planeDetect.get()));

	// Define some publishers
	ros::Publisher planePub = nh.advertise<sensor_msgs::PointCloud2>(
			"/plane", 1);
	ros::Publisher normalPub = nh.advertise<geometry_msgs::PoseStamped>(
			"/plane_normal", 1);
	ros::Publisher distPub = nh.advertise<std_msgs::Float64>(
			"/distance", 1);
	ros::Publisher distFiltPub = nh.advertise<std_msgs::Float64>(
			"/distance_filtered", 1);
	ros::Publisher distVelPub = nh.advertise<std_msgs::Float64>(
			"/distance_velocity", 1);

	// Setup loop rate
	double rate = 10;
	nh.getParam("/detection/rate", rate);
	ROS_INFO("PlaneDetectionNode: Setting rate to %.2f", rate);
	ros::Rate loopRate {rate};
	double dt = 1.0 / rate;

	// Start the main loop
	while(ros::ok())
	{
		ros::spinOnce();
		
		// Do the plane detection
		planeDetect->doPlaneDetection();
		planeDetect->publishPlane(planePub);
		planeDetect->publishNormal(normalPub);
		planeDetect->publishDistanceToPlane(distPub);

		// Filter distance
		planeDetect->filterCurrentDistance(dt);
		planeDetect->publishFiltDist(distFiltPub);
		planeDetect->publishFiltDistVel(distVelPub);

		loopRate.sleep();
	}

	return 0;
}
