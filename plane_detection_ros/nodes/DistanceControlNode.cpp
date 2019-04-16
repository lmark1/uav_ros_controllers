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

/**
 * Initializes distance control node.
 *
 * Default topics for remapping:
 * 		- /distance		- Distance from the UAV to the plane surface
 * 		- /joy			- Joystick topic used for enabling inspection mode
 *		- /real/imu	PID	- IMU topic - realistic
 *		- /sim/odometry	- Odometry topic - simulation
 *
 * ROS parameters:
 * 		- simulation 	- Enable simulation mode
 * 		- rate			- Distance control rate
 */
int main(int argc, char **argv) {

	// Setup the node
	ros::init(argc, argv, "distance_control");
	ros::NodeHandle nh;

	// Get mode parameter
	bool simMode = false;
	double rate = 25;
	nh.getParam("simulation", simMode);
	nh.getParam("rate", rate);

	// Change logging level
	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
		ros::console::levels::Debug))
		ros::console::notifyLoggerLevelsChanged();

	// Initialize distance control object
	std::shared_ptr<DistanceControl> distanceControl
		{new DistanceControl (((simMode) ?
				DistanceControlMode::SIMULATION:
				DistanceControlMode::REAL)) };

	// Setup callbacks
	ros::Subscriber distSub = nh.subscribe("/distance", 1,
			&ControlBase::distanceCb,
			dynamic_cast<ControlBase*>(distanceControl.get()));
	ros::Subscriber joySub = nh.subscribe("/joy", 1,
			&ControlBase::joyCb,
			dynamic_cast<ControlBase*>(distanceControl.get()));

	// Realistic callbacks
	ros::Subscriber imuSub = nh.subscribe("/real/imu", 1,
			&ControlBase::imuCbReal,
			dynamic_cast<ControlBase*>(distanceControl.get()));

	// Simulation callbacks
	ros::Subscriber odomSub = nh.subscribe("/sim/odometry", 1,
			&ControlBase::imuCbSim,
			dynamic_cast<ControlBase*>(distanceControl.get()));

	// Command velocity subscriber
	ros::Subscriber cmdSub = nh.subscribe("/cmd_vel", 1,
			&ControlBase::cmdVelCb,
			dynamic_cast<ControlBase*>(distanceControl.get()));

	// Define publishers
	ros::Publisher statePub = nh.advertise<std_msgs::Int32>(
			"/control_state", 1);

	// Simulation setpoint
	ros::Publisher spPubSim = nh.advertise<geometry_msgs::Vector3>(
			"/sim/attitude_sp", 1);

	// Real setpoint
	ros::Publisher spPubReal = nh.advertise<mavros_msgs::AttitudeTarget>(
			"/real/attitude_sp", 1);

	ROS_DEBUG("Intitialized subscribers.");
	// Initialize configure server
	dynamic_reconfigure::
		Server<plane_detection_ros::DistanceControlParametersConfig>
		confServer;

	// Initialize reconfigure callback
	dynamic_reconfigure::
		Server<plane_detection_ros::DistanceControlParametersConfig>::
		CallbackType
		paramCallback;

	// Setup reconfigure server
	paramCallback = boost::bind(
			&ControlBase::parametersCallback,
			dynamic_cast<ControlBase*>(distanceControl.get()), _1, _2);
	confServer.setCallback(paramCallback);

	ros::Rate loopRate(rate);
	double dt = 1.0 / rate;
	ROS_INFO("Setting rate to %.2f", rate);
	while (ros::ok())
	{
		ros::spinOnce();
		distanceControl->detectStateChange();
		distanceControl->publishState(statePub);
		distanceControl->calculateSetpoint(dt);

		// Publish setpoint based on the inspection status
		if (simMode)
			distanceControl->publishSetpoint(spPubSim);
		else
			distanceControl->publishSetpoint(spPubReal);

		loopRate.sleep();
	}
}





