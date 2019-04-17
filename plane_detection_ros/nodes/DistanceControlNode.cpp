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
 *		- /cmd_vel		- Command velocities from the Joystick input
 */
int main(int argc, char **argv) {

	// Setup the node
	ros::init(argc, argv, "distance_control");
	ros::NodeHandle nh;

	// Get parameters
	bool simMode = false;
	double rate = 25;
	double pidKp = 2;
	double pidKi = 2;
	double pidKd = 2;
	double pidLimLow = -2;
	double pidLimHigh = 2;
	nh.getParam("/control/sim_mode", simMode);
	nh.getParam("/control/rate", rate);
	nh.getParam("/control/pid/kp", pidKp);
	nh.getParam("/control/pid/ki", pidKi);
	nh.getParam("/control/pid/kd", pidKd);
	nh.getParam("/control/pid/lim_low", pidLimLow);
	nh.getParam("/control/pid/lim_high", pidLimHigh);

	// Change logging level
	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
		ros::console::levels::Debug))
		ros::console::notifyLoggerLevelsChanged();

	// Initialize distance control object
	std::shared_ptr<DistanceControl> distanceControl
		{ new DistanceControl { ((simMode) ? 
			DistanceControlMode::SIMULATION : 
			DistanceControlMode::REAL) } };	

	// Set all PID parameters externally
	distanceControl->getPID().set_kp(pidKp);
	distanceControl->getPID().set_ki(pidKi);
	distanceControl->getPID().set_kd(pidKd);
	distanceControl->getPID().set_lim_low(pidLimLow);
	distanceControl->getPID().set_lim_high(pidLimHigh);

	// Setup callbacks
	ros::Subscriber distSub = nh.subscribe("/distance", 1,
			&ControlBase::distanceCb,
			dynamic_cast<ControlBase*>(distanceControl.get()));
	ros::Subscriber joySub = nh.subscribe("/joy", 1,
			&ControlBase::joyCb,
			dynamic_cast<ControlBase*>(distanceControl.get()));

	if (simMode)
		// Simulation callbacks
		ros::Subscriber odomSub = nh.subscribe("/sim/odometry", 1,
			&ControlBase::imuCbSim,
			dynamic_cast<ControlBase*>(distanceControl.get()));
	else
		// Realistic callbacks
		ros::Subscriber imuSub = nh.subscribe("/real/imu", 1,
			&ControlBase::imuCbReal,
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

	// Referent distance publisher
	ros::Publisher distRefPub = nh.advertise<std_msgs::Float64>(
		"/dist_ref", 1);

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
	distanceControl->setReconfigureParameters(confServer);

	// Setup reconfigure server
	paramCallback = boost::bind(
			&ControlBase::parametersCallback,
			dynamic_cast<ControlBase*>(distanceControl.get()), _1, _2);
	confServer.setCallback(paramCallback);

	ros::Rate loopRate(rate);
	double dt = 1.0 / rate;
	ROS_INFO("DistanceControlNode: Setting rate to %.2f", rate);
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

		distanceControl->publishDistanceSetpoint(distRefPub);
		loopRate.sleep();

		/*
		ROS_DEBUG("PID parameters: p=%.2f i=%.2f d=%.2f low=%.2f high=%.2f\n",
			distanceControl->getPID().get_kp(),
			distanceControl->getPID().get_ki(),
			distanceControl->getPID().get_kd(),
			distanceControl->getPID().get_lim_low(),
			distanceControl->getPID().get_lim_high());
		*/
	}
}





