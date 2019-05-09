/*
 * DistanceControlNode.cpp
 *
 *  Created on: Apr 11, 2019
 *      Author: lmark
 */

#include <plane_detection_ros/control/DistanceControl.h>
#include <plane_detection_ros/DistanceControlParametersConfig.h>

// ROS Includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <std_msgs/Int32.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <std_srvs/Empty.h>

/**
 * Initializes distance control node.
 *
 * Default topics for remapping:
 * 		- /distance		- Distance from the UAV to the plane surface
 * 		- /joy			- Joystick topic used for enabling inspection mode
 *		- /real/imu		- IMU topic - realistic
 *		- /real/pos		- Local position topic - realistic
 *		- /real/vel		- Velocity topic - realistic
 *		- /sim/odometry	- Odometry topic - simulation
 */
int main(int argc, char **argv) 
{
	// Setup the node
	ros::init(argc, argv, "distance_control");
	ros::NodeHandle nh;
	// Change logging level
	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
		ros::console::levels::Debug))
		ros::console::notifyLoggerLevelsChanged();

	// Check if sim mode or real
	bool simMode = false;
	nh.getParam("/control/sim_mode", simMode);

	// Initialize distance control object
	std::shared_ptr<dist_control::DistanceControl> distanceControl
		{ new dist_control::DistanceControl { ((simMode) ? 
			dist_control::DistanceControlMode::SIMULATION : 
			dist_control::DistanceControlMode::REAL) } };
	distanceControl->initializeParameters(nh);
	
	// Setup callbacks
	ros::Subscriber distSub = nh.subscribe("/distance", 1,
		&dist_control::DistanceControl::distanceCb,
		distanceControl.get());
	ros::Subscriber distVelSub = nh.subscribe("/distance_velocity", 1,
		&dist_control::DistanceControl::distanceVelCb,
		distanceControl.get());
	// Plane normal CB
	ros::Subscriber planeSub = nh.subscribe("/plane_normal", 1,
		&dist_control::DistanceControl::normalCb,
		distanceControl.get());

	// Joy callback
	ros::Subscriber joySub = nh.subscribe("/joy", 1,
		&joy_control::JoyControl::joyCb,
		distanceControl->getJoyPointer());

	// Simulation callbacks
	ros::Subscriber odomSub = nh.subscribe("/sim/odometry", 1,
		&control_base::ControlBase::odomCbSim,
		distanceControl->getBasePointer());

	// Realistic callbacks
	ros::Subscriber imuSub = nh.subscribe("/real/imu", 1,
		&control_base::ControlBase::imuCbReal,
		distanceControl->getBasePointer());
	ros::Subscriber posSub = nh.subscribe("/real/odometry", 1,
		&control_base::ControlBase::odomCbReal,
		distanceControl->getBasePointer());

	// Define publishers
	ros::Publisher statePub = nh.advertise<std_msgs::Int32>(
		"/control_state", 1);
	ros::Publisher spPubSim = nh.advertise<mav_msgs::RollPitchYawrateThrust>(
		"/sim/rpy_thrust", 1);
	ros::Publisher spPubReal = nh.advertise<mavros_msgs::AttitudeTarget>(
		"/real/attitude_sp", 1);
	ros::Publisher distRefPub = nh.advertise<std_msgs::Float64>(
		"/dist_sp", 1);
	ros::Publisher distVelRefPub = nh.advertise<std_msgs::Float64>(
		"/dist_vel_sp", 1);
	ros::Publisher eulerSpPub = nh.advertise<geometry_msgs::Vector3>(
		"/euler_sp", 1);
	ros::Publisher posSpPub = nh.advertise<geometry_msgs::Vector3>(
		"/carrot_sp", 1);
	ros::Publisher posMvPub = nh.advertise<geometry_msgs::Vector3>(
		"/carrot_mv", 1);
	ros::Publisher velSpPub = nh.advertise<geometry_msgs::Vector3>(
		"/carrot_vel_sp", 1);
	ros::Publisher velMvPub = nh.advertise<geometry_msgs::Vector3>(
		"/carrot_vel_mv", 1);

	// Initialize distance reconfigure server
	// TODO: Move these blocks to appropriate classes
	boost::recursive_mutex distConfigMutex;
	dynamic_reconfigure::
		Server<plane_detection_ros::DistanceControlParametersConfig>
		distConfigServer {distConfigMutex, ros::NodeHandle("distance_config")};
	dynamic_reconfigure::
		Server<plane_detection_ros::DistanceControlParametersConfig>::CallbackType
		distParamCallback;
	plane_detection_ros::DistanceControlParametersConfig distConfig;
	distanceControl->setReconfigureParameters(distConfig);
	distConfigServer.updateConfig(distConfig);
	distParamCallback = boost::bind(
		&dist_control::DistanceControl::parametersCallback,
		distanceControl.get(), _1, _2);
	distConfigServer.setCallback(distParamCallback);

	// Initialize position reconfigure server
	boost::recursive_mutex posConfigMutex;
	dynamic_reconfigure::
		Server<plane_detection_ros::PositionControlParametersConfig>
		posConfigServer {posConfigMutex, ros::NodeHandle("position_config")};
	dynamic_reconfigure::
		Server<plane_detection_ros::PositionControlParametersConfig>::CallbackType
		posParamCallback;
	plane_detection_ros::PositionControlParametersConfig posConfig;
	distanceControl->getCarrotPointer()->setReconfigureParameters(posConfig);
	posConfigServer.updateConfig(posConfig);
	posParamCallback = boost::bind(
		&carrot_control::CarrotControl::parametersCallback,
		distanceControl->getCarrotPointer(), _1, _2);
	posConfigServer.setCallback(posParamCallback);

	// Setup loop rate
	double rate = 25;
	nh.getParam("/control/rate", rate);
	ros::Rate loopRate(rate);
	double dt = 1.0 / rate;
	ROS_INFO("DistanceControlNode: Setting rate to %.2f", rate);
	
	double timeElapsed = 0;
	bool holdPosition = false;
	double holdTime = 5;
	nh.getParam("/control/hold_time", holdTime);
	ROS_INFO("DistanceControlNode: Setting hold time to %.2f", holdTime);
	
	// Initialize override service here
	ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>(
		"magnet/override_ON");
	std_srvs::Empty emptyMessage;
	bool serviceCalled = false;

	// TODO: Make DistanceControl have callback activations for InspectionMode 
	// TODO: Move InpsectionIndices to SequenceControl
	// Publish offset constantly, when standing still offset iz 0, when moving offset is constant, 
	// not in sequence offset is from joy
	// Move all the deadzones to JoyControl

	// Start the main loop
	while (ros::ok())
	{
		ros::spinOnce();
		distanceControl->detectStateChange();
		distanceControl->detectSequenceChange();
		distanceControl->publishState(statePub);
		
		// Do regular "Manual" Inspection when sequence is not set
		if (distanceControl->inInspectionState() && 
			distanceControl->getSequence() == dist_control::Sequence::NONE)
		{
			holdPosition = false;
			serviceCalled = false;
			timeElapsed = 0;
			distanceControl->calculateInspectionSetpoint(dt);
		}

		// Do "Sequence" Inpsection when sequence is set
		else if (distanceControl->inInspectionState() &&
			distanceControl->getSequence() != dist_control::Sequence::NONE)	
		{
			
			// If target is reached atleast once, hold position
			if (!holdPosition && distanceControl->seqTargetReached())
				holdPosition = true;

			// If hold position is activated, hold position for fixed time
			if (holdPosition && timeElapsed < holdTime)
			{
				timeElapsed += dt;
				distanceControl->updateCarrotZ();
				distanceControl->doDistanceControl(dt);
				
				// Call the override service
				if (!serviceCalled)
				{
					ROS_INFO("main() - Calling overide service");
					serviceCalled = true;
					client.call(emptyMessage);
				}
			}

			// When done holding position, go to new sequence target.
			else
			{
				holdPosition = false;
				serviceCalled = false;
				timeElapsed = 0;
				distanceControl->calculateSequenceSetpoint(dt);
			}
				
		}			

		// If not in inspection state go to attitude control
		else
		{
			holdPosition = false;
			serviceCalled = false;
			timeElapsed = 0;
			distanceControl->calculateManualSetpoint(dt);
		}
			

		// Publish simulation setpoint
		if (simMode)
			distanceControl->publishAttSp(spPubSim);

		// Publish real setpoint
		else
			distanceControl->publishAttSp(spPubReal);

		// Publish distance and angle setpoints
		distanceControl->publishDistSp(distRefPub);
		distanceControl->publishDistVelSp(distVelRefPub);

		// Publish carrot setpoints
		distanceControl->publishEulerSp(eulerSpPub);
		distanceControl->publishPosSp(posSpPub);
		distanceControl->publishVelSp(velSpPub);
		distanceControl->publishPosMv(posMvPub);
		distanceControl->publishVelMv(velMvPub);
		
		loopRate.sleep();
	}
}





