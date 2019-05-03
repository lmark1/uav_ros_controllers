// ROS Includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <std_msgs/Int32.h>
#include <mav_msgs/RollPitchYawrateThrust.h>

// Own includes
#include <plane_detection_ros/control/CarrotControl.h>

int main(int argc, char **argv) {

	// Setup the node
	ros::init(argc, argv, "distance_control");
	ros::NodeHandle nh;
	// Change logging level
	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
		ros::console::levels::Debug))
		ros::console::notifyLoggerLevelsChanged();

    // Initialize distance control object
	std::shared_ptr<carrot_control::CarrotControl> carrotControl
		{ new carrot_control::CarrotControl };
	carrotControl->initializeParameters(nh);
	
    // Joy callback
	ros::Subscriber joySub = nh.subscribe("/joy", 1,
		&joy_control::JoyControl::joyCb,
		carrotControl->getJoyPointer());

	// Simulation callbacks
	ros::Subscriber odomSub = nh.subscribe("/sim/odometry", 1,
		&control_base::ControlBase::odomCbSim,
		carrotControl->getBasePointer());

	// Realistic callbacks
	ros::Subscriber imuSub = nh.subscribe("/real/imu", 1,
		&control_base::ControlBase::imuCbReal,
		carrotControl->getBasePointer());
	ros::Subscriber posSub = nh.subscribe("/real/odometry", 1,
		&control_base::ControlBase::odomCbReal,
		carrotControl->getBasePointer());
    
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

    // Initialize position reconfigure server
	boost::recursive_mutex posConfigMutex;
	dynamic_reconfigure::
		Server<plane_detection_ros::PositionControlParametersConfig>
		posConfigServer {posConfigMutex, ros::NodeHandle("position_config")};
	dynamic_reconfigure::
		Server<plane_detection_ros::PositionControlParametersConfig>::CallbackType
		posParamCallback;
	plane_detection_ros::PositionControlParametersConfig posConfig;
	carrotControl->getCarrotPointer()->setReconfigureParameters(posConfig);
	posConfigServer.updateConfig(posConfig);
	posParamCallback = boost::bind(
		&carrot_control::CarrotControl::parametersCallback,
		carrotControl->getCarrotPointer(), _1, _2);
	posConfigServer.setCallback(posParamCallback);

    // Setup loop rate
	double rate = 25;
	nh.getParam("/control/rate", rate);
	ros::Rate loopRate(rate);
	double dt = 1.0 / rate;
	ROS_INFO("carrotControlNode: Setting rate to %.2f", rate);

    // Check if sim mode or real
	bool simMode = false;
	nh.getParam("/control/sim_mode", simMode);

    // Start the main loop
	while (ros::ok())
	{
		ros::spinOnce();
        carrotControl->updateCarrot();
        carrotControl->calculateAttThrustSp(dt);
		
		// Publish setpoint based on the control mode
		if (simMode)
			carrotControl->publishAttitudeSim(
                spPubSim, carrotControl->getThrustScale());        
		else
			carrotControl->publishAttitudeReal(spPubReal);

		// Publish carrot setpoints
        carrotControl->publishEulerSp(eulerSpPub);
		carrotControl->publishPosSp(posSpPub);
		carrotControl->publishVelSp(velSpPub);
		carrotControl->publishPosMv(posMvPub);
		carrotControl->publishVelMv(velMvPub);
		
		loopRate.sleep();
	}
}