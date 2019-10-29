#ifndef VISUAL_SERVO_STATE_MACHINE_H
#define VISUAL_SERVO_STATE_MACHINE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <dynamic_reconfigure/server.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <uav_ros_control/VisualServoStateMachineParametersConfig.h>

namespace uav_reference 
{

typedef uav_ros_control::VisualServoStateMachineParametersConfig vssm_param_t;
#define VSSM_DYN_RECONF             "vs_state_machine"
#define PARAM_MIN_ERROR             "visual_servo/state_machine/min_error"
#define PARAM_TOUCHDOWN_HEIGHT      "visual_servo/state_machine/touchdown_height"
#define PARAM_TOUCHDOWN_DELTA       "visual_servo/state_machine/touchdown_delta"
#define PARAM_TOUCHDOWN_DURATION    "visual_servo/state_machine/touchdown_duration"
#define PARAM_RATE                  "visual_servo/state_machine/rate"
#define VS_ACTIVE   "ON"
#define VS_INACTIVE "OFF"

enum VisualServoState {
    OFF,
    BRICK_ALIGNMENT,
    DESCENT,
    TOUCHDOWN
};

class VisualServoStateMachine
{

public:

VisualServoStateMachine(ros::NodeHandle& nh)
{   
	initializeParameters(nh);
    // Define Publishers
    _pubVisualServoPoseSp = 
        nh.advertise<geometry_msgs::PoseStamped>("visual_servo/sm/pose", 1);

    // Define Subscribers
    _subOdom =
        nh.subscribe("odometry", 1, &uav_reference::VisualServoStateMachine::odomCb, this);
    _subTargetError =  
        nh.subscribe("visual_servo/sm/error", 1, &uav_reference::VisualServoStateMachine::errorCb, this);
    _subVSStatus = nh.subscribe("visual_servo/status", 1, &uav_reference::VisualServoStateMachine::statusCb, this);

    // Setup dynamic reconfigure server
	vssm_param_t  vssmConfig;
	setVSSMParameters(vssmConfig);
	_vssmConfigServer.updateConfig(vssmConfig);
	_vssmParamCallback = boost::bind(
		&uav_reference::VisualServoStateMachine::vssmParamCb, this, _1, _2);
	_vssmConfigServer.setCallback(_vssmParamCallback);

    // Setup brick pickup service callback
    _serviceBrickPickup = nh.advertiseService(
			"brick_pickup",
			&uav_reference::VisualServoStateMachine::brickPickupServiceCb,
			this);

    // Initialize visual servo client caller
    _vsClienCaller = nh.serviceClient<std_srvs::Empty::Request, std_srvs::SetBool::Response>("visual_servo");
}

~VisualServoStateMachine()
{}

bool brickPickupServiceCb(std_srvs::Empty::Request& request, std_srvs::SetBool::Response& response)
{
    if (_brickPickupActivated)
    {
        ROS_FATAL("VisualServoStateMachine::brickPickupServiceCb - brick pickup is already active.");
        response.success = false;
        response.message = "Brick pickup is already active";
        return true;
    }

    // Try calling visual servo
    std_srvs::Empty::Request req;
    std_srvs::SetBool::Response resp;
    if (!_vsClienCaller.call(req, resp))
    {
        ROS_FATAL("VisualServoStateMachine::brickPickupServiceCb - calling visual servo failed.");
        response.success = false;
        response.message = "Calling visual servo failed.";
        return true;
    }

    // Visual servo successfully activated
    ROS_INFO("VisualServoStateMachine::brickPickupServiceCb - brick pickup activated.");
    response.success = true;
    response.message = "Brick pickup activated.";
    return true;
}

void vssmParamCb(vssm_param_t& configMsg,uint32_t level)
{
    ROS_WARN("VisualServoStateMachine::vssmParamCb()");
    _minTargetError = configMsg.min_error;
    _touchdownHeight = configMsg.touchdown_height;
    _touchdownDelta = configMsg.touchdown_delta;
    _touchdownDuration = configMsg.touchdown_duration;
}

void setVSSMParameters(vssm_param_t& config)
{
    config.min_error = _minTargetError;
    config.touchdown_delta = _touchdownDelta;
    config.touchdown_duration = _touchdownDuration;
    config.touchdown_height = _touchdownHeight;
}

void initializeParameters(ros::NodeHandle& nh)
{
    ROS_INFO("VisualServoStateMachine::initializeParameters()");
    bool initialized = nh.getParam(PARAM_MIN_ERROR, _minTargetError)
		&& nh.getParam(PARAM_TOUCHDOWN_HEIGHT, _touchdownHeight)
		&& nh.getParam(PARAM_TOUCHDOWN_DURATION, _touchdownDuration)
        && nh.getParam(PARAM_TOUCHDOWN_DELTA, _touchdownDelta)
        && nh.getParam(PARAM_RATE, _rate);
    ROS_INFO("Node rate: %.2f", _rate);
    ROS_INFO("Minimum target error: %.2f", _minTargetError);
    ROS_INFO("Touchdown height: %.2f", _touchdownHeight);
    ROS_INFO("Touchdown duration: %.2f", _touchdownDuration);
    ROS_INFO("Touchdown delta: %.2f", _touchdownDelta);
    if (!initialized)
	{
		ROS_FATAL("VisualServoStateMachine::initializeParameters() - failed to initialize parameters");
		throw std::runtime_error("VisualServoStateMachine parameters not properly initialized.");
	}
}

void updateState()
{
    // If visual servo is inactive, deactivate state machine
    if (_vsStatus == VS_INACTIVE)
    {
        ROS_INFO("VSSM::updateStatus - Visual servo is inactive.");
        _currentState = VisualServoState::OFF;
        _brickPickupActivated = false;
        ROS_INFO("VSSM::updateStatus - OFF State activated.");
        return;
    }

    // If brick pickup is activate start brick alignment first
    if (_brickPickupActivated && _currentState == VisualServoState::OFF)
    {
        ROS_INFO("VSSM::updateStatus - Brick pickup requested");
        _currentState = VisualServoState::BRICK_ALIGNMENT;
        ROS_INFO("VSSM::updateStatus - BRICK_ALIGNMENT state activated.");
        return;
    }

    // If brick alignemnt is activated and target error is withing range start descent
    if (_currTargetError < _minTargetError && _currentState == VisualServoState::BRICK_ALIGNMENT)
    {
        _currentState = VisualServoState::DESCENT;
        ROS_INFO("VSSM::updateStatus - DESCENT state activated");
        return;
    }

    // if height is below touchdown treshold start touchdown TODO
}

void statusCb(const std_msgs::StringConstPtr& msg)
{
    _vsStatus = msg->data;
}

void odomCb(const nav_msgs::OdometryConstPtr& msg)
{
    _currOdom = *msg;
}

void errorCb(const std_msgs::Float64ConstPtr& msg)
{
    _currTargetError = msg->data;
}

void run()
{
    ros::Rate loopRate(_rate);
	while (ros::ok())
	{
		ros::spinOnce();

        loopRate.sleep();
    }
}

private:

    double _rate = 50;

    /* Service brick pickup */
	ros::ServiceServer _serviceBrickPickup;
    bool _brickPickupActivated = false;
    VisualServoState _currentState = VisualServoState::OFF;
    
    /* Client for calling visual servo */
    ros::ServiceClient _vsClienCaller;

    // Pose publisher
    ros::Publisher _pubVisualServoPoseSp;
    geometry_msgs::PoseStamped _currPose;

    /* Odometry subscriber */
    ros::Subscriber _subOdom;
    nav_msgs::Odometry _currOdom;

    /* Target error subscriber */
    ros::Subscriber _subTargetError;
    double _currTargetError = 1e5;
    double _minTargetError;

    /* VS status subscriber */
    ros::Subscriber _subVSStatus;
    std::string _vsStatus = VS_INACTIVE;
    
    /* Touchdown mode parameters */
    double _touchdownHeight, _touchdownDelta, _touchdownDuration;

    /* Define Dynamic Reconfigure parameters */
    boost::recursive_mutex _vssmConfigMutex;
    dynamic_reconfigure::Server<vssm_param_t>
        _vssmConfigServer {_vssmConfigMutex, ros::NodeHandle(VSSM_DYN_RECONF)};
    dynamic_reconfigure::Server<vssm_param_t>::CallbackType _vssmParamCallback;
};
}

#endif /* VISUAL_SERVO_STATE_MACHINE_H */