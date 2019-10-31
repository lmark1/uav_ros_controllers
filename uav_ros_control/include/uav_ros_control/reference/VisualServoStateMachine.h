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
#include <uav_ros_control_msgs/VisualServoProcessValues.h>
#include <uav_ros_control/filters/NonlinearFilters.h>

namespace uav_reference 
{

typedef uav_ros_control::VisualServoStateMachineParametersConfig vssm_param_t;
#define VSSM_DYN_RECONF             "vs_state_machine"
#define PARAM_MIN_ERROR             "visual_servo/state_machine/min_error"
#define PARAM_MIN_YAW_ERROR         "visual_servo/state_machine/min_yaw_error"
#define PARAM_TOUCHDOWN_HEIGHT      "visual_servo/state_machine/touchdown_height"
#define PARAM_TOUCHDOWN_DELTA       "visual_servo/state_machine/touchdown_delta"
#define PARAM_TOUCHDOWN_DURATION    "visual_servo/state_machine/touchdown_duration"
#define PARAM_RATE                  "visual_servo/state_machine/rate"
#define PARAM_DESCENT_SPEED         "visual_servo/state_machine/descent_speed"
#define PARAM_OFF1_X                "visual_servo/state_machine/offset_x_1"
#define PARAM_OFF2_X                "visual_servo/state_machine/offset_x_2"
#define PARAM_OFF1_Y                "visual_servo/state_machine/offset_y_1"
#define PARAM_OFF2_Y                "visual_servo/state_machine/offset_y_2"


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
    _pubVisualServoFeed = 
        nh.advertise<uav_ros_control_msgs::VisualServoProcessValues>("visual_servo/process_value", 1);
    _pubOffsetX = nh.advertise<std_msgs::Float64>("visual_servo/offset_x", 1);
    _pubOffsetY = nh.advertise<std_msgs::Float64>("visual_servo/offset_y", 1);

    // Define Subscribers
    _subOdom =
        nh.subscribe("odometry", 1, &uav_reference::VisualServoStateMachine::odomCb, this);
    _subTargetError =  
        nh.subscribe("visual_servo/target_error", 1, &uav_reference::VisualServoStateMachine::targetErrorCb, this);
    _subYawError = 
        nh.subscribe("visual_servo/yaw_error", 1, &uav_reference::VisualServoStateMachine::yawErrorCb, this); 
    _subVSStatus = 
        nh.subscribe("visual_servo/status", 1, &uav_reference::VisualServoStateMachine::statusCb, this);
    _subOffsetX = 
        nh.subscribe("visual_servo/x_error", 1, &uav_reference::VisualServoStateMachine::xErrorCb, this);
    _subOffsetY =
        nh.subscribe("visual_servo/y_error", 1, &uav_reference::VisualServoStateMachine::yErrorCb, this);

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

    // If VS is already active for some reason ...
    if (_vsStatus == VS_ACTIVE)
    {
        ROS_INFO("Visual servo is already active.");
        _brickPickupActivated = true;
        response.message = true;
        response.message = "Brick pickup activated";
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

    if (resp.success)
    {
        // Visual servo successfully activated
        ROS_INFO("VisualServoStateMachine::brickPickupServiceCb - brick pickup activated.");
        response.success = true;
        response.message = "Brick pickup activated.";
        _brickPickupActivated = true;
        return true;
    }
    
    ROS_INFO("VisualServoStateMachine::brickPickupServiceCb - unable to activate brick pickup.");
    response.success = false;
    response.message = "Brick pickup not activated";
    
    return true;
}

void vssmParamCb(vssm_param_t& configMsg,uint32_t level)
{
    ROS_WARN("VisualServoStateMachine::vssmParamCb()");
    _minTargetError = configMsg.min_error;
    _minYawError = configMsg.min_yaw_error;
    _touchdownHeight = configMsg.touchdown_height;
    _touchdownDelta = configMsg.touchdown_delta;
    _touchdownDuration = configMsg.touchdown_duration;
    _offset_x_1 = configMsg.x_offset_1;
    _offset_x_2 = configMsg.x_offset_2;
    _offset_y_1 = configMsg.y_offset_1;
    _offset_y_2 = configMsg.y_offset_2;
}

void setVSSMParameters(vssm_param_t& config)
{
    config.min_error = _minTargetError;
    config.min_yaw_error = _minYawError;
    config.x_offset_1 = _offset_x_1;
    config.x_offset_2 = _offset_x_2;
    config.y_offset_1 = _offset_y_1;
    config.y_offset_2 = _offset_y_2;
    config.touchdown_delta = _touchdownDelta;
    config.touchdown_duration = _touchdownDuration;
    config.touchdown_height = _touchdownHeight;
}

void initializeParameters(ros::NodeHandle& nh)
{
    ROS_INFO("VisualServoStateMachine::initializeParameters()");
    bool initialized = nh.getParam(PARAM_MIN_ERROR, _minTargetError)
        && nh.getParam(PARAM_MIN_YAW_ERROR, _minYawError)
		&& nh.getParam(PARAM_TOUCHDOWN_HEIGHT, _touchdownHeight)
		&& nh.getParam(PARAM_TOUCHDOWN_DURATION, _touchdownDuration)
        && nh.getParam(PARAM_TOUCHDOWN_DELTA, _touchdownDelta)
        && nh.getParam(PARAM_DESCENT_SPEED, _descentSpeed)
        && nh.getParam(PARAM_OFF1_X, _offset_x_1)
        && nh.getParam(PARAM_OFF2_X, _offset_x_2)
        && nh.getParam(PARAM_OFF1_Y, _offset_y_1)
        && nh.getParam(PARAM_OFF2_Y, _offset_y_2)
        && nh.getParam(PARAM_RATE, _rate);
    ROS_INFO("Node rate: %.2f", _rate);
    ROS_INFO("Minimum target error: %.2f", _minTargetError);
    ROS_INFO("Minimum yaw error: %.2f", _minYawError);
    ROS_INFO("Descent speed: %.2f", _descentSpeed);
    ROS_INFO("Touchdown height: %.2f", _touchdownHeight);
    ROS_INFO("Touchdown duration: %.2f", _touchdownDuration);
    ROS_INFO("Touchdown delta: %.2f", _touchdownDelta);
    ROS_INFO("X offsets: %.2f, %.2f", _offset_x_1, _offset_x_2);
    ROS_INFO("Y offsets: %.2f, %.2f", _offset_y_1, _offset_y_2);
    if (!initialized)
	{
		ROS_FATAL("VisualServoStateMachine::initializeParameters() - failed to initialize parameters");
		throw std::runtime_error("VisualServoStateMachine parameters not properly initialized.");
	}
}

void updateState()
{
    // If visual servo is inactive, deactivate state machine
    if (_vsStatus == VS_INACTIVE || !_brickPickupActivated)
    {
        ROS_INFO("VSSM::updateStatus - Visual servo is inactive.");
        _currentState = VisualServoState::OFF;
        _brickPickupActivated = false;
        ROS_INFO("VSSM::updateStatus - OFF State activated.");
        return;
    }

    // If brick pickup is activate start brick alignment first
    if (_currentState == VisualServoState::OFF && _brickPickupActivated)
    {
        ROS_INFO("VSSM::updateStatus - Brick pickup requested");
        _currHeightReference = _currOdom.pose.pose.position.z;
        _currentState = VisualServoState::BRICK_ALIGNMENT;
        ROS_INFO("VSSM::updateStatus - BRICK_ALIGNMENT state activated with height: %2f.", _currHeightReference);
        return;
    }

    // If brick alignemnt is activated and target error is withing range start descent
    if (_currentState == VisualServoState::BRICK_ALIGNMENT &&
        _currTargetError < _minTargetError && 
        _currYawError < _minYawError)
    {
        _currentState = VisualServoState::DESCENT;
        ROS_INFO("VSSM::updateStatus - DESCENT state activated");
        return;
    }

    // if height is below touchdown treshold start touchdown
    if (_currentState == VisualServoState::DESCENT && 
        _currOdom.pose.pose.position.z <= _touchdownHeight)
    {
        _currentState = VisualServoState::TOUCHDOWN;
        _touchdownTime = 0;
        _currHeightReference = _currOdom.pose.pose.position.z;
        ROS_INFO("VSSM::UpdateStatus - TOUCHDOWN state activated");
        return;
    }

    // If touchdown time is exceeded, touchdown state is considered finished
    if (_currentState == VisualServoState::TOUCHDOWN &&
        _touchdownTime >= _touchdownDuration)
    {
        ROS_INFO("VSSM::updateStatus - Touchdown duration finished.");
        if (_vsStatus == VS_INACTIVE)
        {
            _currentState = VisualServoState::OFF;
            ROS_INFO("VSSM::updateStatus - OFF state activated. ");
            _brickPickupActivated = false;
            ROS_INFO("VSSM::updateStatus - Brick pickup finished.");
            return;
        }

        // Attempt to turn off visual servo
        std_srvs::Empty::Request req;
        std_srvs::SetBool::Response resp;
        if (!_vsClienCaller.call(req, resp))
        {
            ROS_FATAL("VSSM::updateStatus - calling visual servo failed.");
            return;
        }

        if (resp.success)
        {
            ROS_INFO("VSSM::updateStatus - visual servo successfully deactivated");
            // Visual servo successfully activated
            _currentState = VisualServoState::OFF;
            ROS_INFO("VSSM::updateStatus - OFF state activated. ");
            _brickPickupActivated = false; 
            ROS_INFO("VSSM::updateStatus - Brick pickup finished.");
            return;
        }    
    }
}   

void publishVisualServoSetpoint(double dt)
{
    switch (_currentState)
    {
        case VisualServoState::OFF :
            _currVisualServoFeed.x = 0;
            _currVisualServoFeed.y = 0;
            _currVisualServoFeed.z = 0;
            _currVisualServoFeed.yaw = 0;
            break;
        
        case VisualServoState::BRICK_ALIGNMENT : 
            _currVisualServoFeed.x = _currOdom.pose.pose.position.x;
            _currVisualServoFeed.y = _currOdom.pose.pose.position.y;
            _currVisualServoFeed.x = _currHeightReference;
            _currVisualServoFeed.yaw = util::calculateYaw(
                _currOdom.pose.pose.orientation.x,
                _currOdom.pose.pose.orientation.y,
                _currOdom.pose.pose.orientation.z,
                _currOdom.pose.pose.orientation.w);
            break;
        
        case VisualServoState::DESCENT : 
            _currVisualServoFeed.x = _currOdom.pose.pose.position.x;
            _currVisualServoFeed.y = _currOdom.pose.pose.position.y;
            _currVisualServoFeed.z = _currHeightReference - _descentSpeed * dt;
            _currHeightReference = _currVisualServoFeed.z;
            _currVisualServoFeed.yaw = 0;
            break;
        
        case VisualServoState::TOUCHDOWN : 
            _currVisualServoFeed.x = 0;
            _currVisualServoFeed.y = 0;
            double dz = 2 * _touchdownDelta / _touchdownDuration * dt;
            if (_touchdownTime < _touchdownDuration/2.0)
                _currVisualServoFeed.z = _currHeightReference - dz;
            else
                _currVisualServoFeed.z = _currHeightReference + dz;
            _currVisualServoFeed.yaw = 0;
            _touchdownTime +=dt;
            break;
    }

    _currVisualServoFeed.header.stamp = ros::Time::now();
    _pubVisualServoFeed.publish(_currVisualServoFeed);

    std_msgs::Float64 xOffset, yOffset;

}

void xErrorCb(const std_msgs::Float64ConstPtr &msg) 
{
    double offset_x_0 = _offset_x_1 + (_offset_x_1 - _offset_x_2);
    double offset_x = (_offset_x_2 - _offset_x_1) * _currOdom.pose.pose.position.z + offset_x_0;
    
    // the offset should have the same sign as the offsets at 2 and 1
    if (offset_x * _offset_x_1 < 0 ) offset_x = 0;

    std_msgs::Float64 newMsg;
    newMsg.data = offset_x;
    _pubOffsetX.publish(newMsg);
}

void yErrorCb(const std_msgs::Float64ConstPtr &msg) 
{
    double offset_y_0 = _offset_y_1 + (_offset_y_1 - _offset_y_2);
    double offset_y = (_offset_y_2 - _offset_y_1) * _currOdom.pose.pose.position.z + offset_y_0;

    // the offset should have the same sign as the offsets at 2 and 1
    if (offset_y * _offset_y_1 < 0 ) offset_y = 0;

    std_msgs::Float64 newMsg;
    newMsg.data = offset_y;
    _pubOffsetY.publish(newMsg);
}

void statusCb(const std_msgs::StringConstPtr& msg)
{
    _vsStatus = msg->data;
}

void odomCb(const nav_msgs::OdometryConstPtr& msg)
{
    _currOdom = *msg;
}

void targetErrorCb(const std_msgs::Float64ConstPtr& msg)
{
    _currTargetError = msg->data;
}

void yawErrorCb(const std_msgs::Float64ConstPtr& msg)
{
    _currYawError = msg->data;
}

void run()
{
    ros::Rate loopRate(_rate);
    double dt = 1.0 / _rate;
	while (ros::ok())
	{
		ros::spinOnce();
        updateState();
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

    /* Offset subscriber and publisher */
    ros::Subscriber _subOffsetX;
    ros::Subscriber _subOffsetY;
    ros::Publisher _pubOffsetX;
    ros::Publisher _pubOffsetY;

    /* Pose publisher */
    ros::Publisher _pubVisualServoFeed;
    uav_ros_control_msgs::VisualServoProcessValues _currVisualServoFeed;

    /* Odometry subscriber */
    ros::Subscriber _subOdom;
    nav_msgs::Odometry _currOdom;

    /* Target error subscriber */
    ros::Subscriber _subTargetError;
    double _currTargetError = 1e5;
    double _minTargetError;

    /* Yaw error subscriber */
    ros::Subscriber _subYawError;
    double _currYawError = 1e5;
    double _minYawError;

    /* VS status subscriber */
    ros::Subscriber _subVSStatus;
    std::string _vsStatus = VS_INACTIVE;
    
    /* Touchdown mode parameters */
    double _touchdownHeight, _touchdownDelta, _touchdownDuration, _touchdownTime;
    double _currHeightReference, _descentSpeed; 
    double _offset_x_1, _offset_x_2, _offset_y_1, _offset_y_2;

    /* Define Dynamic Reconfigure parameters */
    boost::recursive_mutex _vssmConfigMutex;
    dynamic_reconfigure::Server<vssm_param_t>
        _vssmConfigServer {_vssmConfigMutex, ros::NodeHandle(VSSM_DYN_RECONF)};
    dynamic_reconfigure::Server<vssm_param_t>::CallbackType _vssmParamCallback;
};
}

#endif /* VISUAL_SERVO_STATE_MACHINE_H */