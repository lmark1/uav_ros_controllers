/*
 * DistanceControlNode.cpp
 *
 *  Created on: Apr 11, 2019
 *      Author: lmark
 */

#include "plane_detection_ros/control/DistanceControl.h"
#include <plane_detection_ros/DistanceControlParametersConfig.h>

// ROS Includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <std_msgs/Int32.h>
#include <mav_msgs/RollPitchYawrateThrust.h>

/**
 * Initializes distance control node.
 *
 * Default topics for remapping:
 * 		- /distance		- Distance from the UAV to the plane surface
 * 		- /joy			- Joystick topic used for enabling inspection mode
 *		- /real/imu		- IMU topic - realistic
 *		- /real/pos		- Local position topic - realistic
 *		- /sim/odometry	- Odometry topic - simulation
 */
int main(int argc, char **argv) {

	// Setup the node
	ros::init(argc, argv, "distance_control");
	ros::NodeHandle nh;
	// Change logging level
	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
		ros::console::levels::Info))
		ros::console::notifyLoggerLevelsChanged();

	// Check if sim mode or real
	bool simMode = false;
	nh.getParam("/control/sim_mode", simMode);

	// Initialize distance control object
	std::shared_ptr<DistanceControl> distanceControl
		{ new DistanceControl { ((simMode) ? 
			DistanceControlMode::SIMULATION : 
			DistanceControlMode::REAL) } };
	distanceControl->initializeParameters(nh);
	
	// Setup callbacks
	ros::Subscriber distSub = nh.subscribe("/distance", 1,
			&ControlBase::distanceCb,
			dynamic_cast<ControlBase*>(distanceControl.get()));
	ros::Subscriber distVelSub = nh.subscribe("/distance_velocity", 1,
			&ControlBase::distanceVelCb,
			dynamic_cast<ControlBase*>(distanceControl.get()));
	ros::Subscriber joySub = nh.subscribe("/joy", 1,
			&ControlBase::joyCb,
			dynamic_cast<ControlBase*>(distanceControl.get()));

	// Simulation callbacks
	ros::Subscriber odomSub = nh.subscribe("/sim/odometry", 1,
		&ControlBase::imuCbSim,
		dynamic_cast<ControlBase*>(distanceControl.get()));

	// Realistic callbacks
	ros::Subscriber imuSub = nh.subscribe("/real/imu", 1,
		&ControlBase::imuCbReal,
		dynamic_cast<ControlBase*>(distanceControl.get()));
	ros::Subscriber posSub = nh.subscribe("/real/pos", 1,
		&ControlBase::imuCbReal,
		dynamic_cast<ControlBase*>(distanceControl.get()));
		
	// Plane normal CB
	ros::Subscriber planeSub = nh.subscribe("/plane_normal", 1,
		&ControlBase::normalCb,
		dynamic_cast<ControlBase*>(distanceControl.get()));

	// Define publishers
	ros::Publisher statePub = nh.advertise<std_msgs::Int32>(
			"/control_state", 1);
	// Simulation setpointconfServer
	ros::Publisher spPubSim = nh.advertise<mav_msgs::RollPitchYawrateThrust>(
			"/sim/rpy_thrust", 1);
	// Real setpoint
	ros::Publisher spPubReal = nh.advertise<mavros_msgs::AttitudeTarget>(
			"/real/attitude_sp", 1);
	// Referent distance publisher
	ros::Publisher distRefPub = nh.advertise<std_msgs::Float64>(
		"/dist_sp", 1);
	ros::Publisher distVelRefPub = nh.advertise<std_msgs::Float64>(
		"/dist_vel_sp", 1);
	// Add euler_sp publisher
	ros::Publisher eulerSpPub = nh.advertise<geometry_msgs::Vector3>(
		"/euler_sp", 1);

	boost::recursive_mutex config_mutex;
	// Initialize configure server
	dynamic_reconfigure::
		Server<plane_detection_ros::DistanceControlParametersConfig>
		confServer {config_mutex};
	// Initialize reconfigure callback
	dynamic_reconfigure::
		Server<plane_detection_ros::DistanceControlParametersConfig>::
		CallbackType
		paramCallback;

	// Set initial parameters
	plane_detection_ros::DistanceControlParametersConfig config;
	confServer.setConfigDefault(config);
	distanceControl->setReconfigureParameters(config);
	confServer.updateConfig(config);

	// Setup reconfigure server
	paramCallback = boost::bind(
			&ControlBase::parametersCallback,
			dynamic_cast<ControlBase*>(distanceControl.get()), _1, _2);
	confServer.setCallback(paramCallback);

	// Setup loop rate
	double rate = 25;
	nh.getParam("/control/rate", rate);
	ros::Rate loopRate(rate);
	double dt = 1.0 / rate;
	ROS_INFO("DistanceControlNode: Setting rate to %.2f", rate);

	// Start the main loop
	while (ros::ok())
	{
		ros::spinOnce();
		distanceControl->detectStateChange();
		distanceControl->publishState(statePub);
		distanceControl->calculateSetpoint(dt);

		// Publish setpoint based on the inspection status
		if (simMode)
			distanceControl->publishAttSp(spPubSim);
		else
			distanceControl->publishAttSp(spPubReal);

		distanceControl->publishDistSp(distRefPub);
		distanceControl->publishDistVelSp(distVelRefPub);
		distanceControl->publishEulerSp(eulerSpPub);
		loopRate.sleep();
	}
}





