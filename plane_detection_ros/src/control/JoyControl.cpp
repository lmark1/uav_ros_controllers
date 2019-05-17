#include <plane_detection_ros/control/JoyControl.h>
#include <uav_ros_control/NonlinearFilters.h>

joy_control::JoyControl::JoyControl(ros::NodeHandle& nh) :
	_controlIndices (new joy_struct::ControlIndices),
	_attitudeScales (new joy_struct::ScaleWeights),
	_positionScales (new joy_struct::ScaleWeights)
{
	// Initialize JoyMsg
	_joyMsg.axes = std::vector<float> (10, 0.0);
	_joyMsg.buttons = std::vector<int> (10, 0);	

	// Initialize class parameters
	joy_control::JoyControl::initializeParameters(nh);

	// Initialize Joy subscriber
	_subJoy = nh.subscribe("/joy", 1, &joy_control::JoyControl::joyCb, this);
}

joy_control::JoyControl::~JoyControl()
{
}

void joy_control::JoyControl::joyCb(const sensor_msgs::JoyConstPtr& message)
{
    _joyMsg = *message;
}

double joy_control::JoyControl::getRollSpManual()
{
	return 	_joyMsg.axes[_controlIndices->AXIS_LINEAR_Y] * _attitudeScales->LINEAR_Y ;
}

double joy_control::JoyControl::getPitchSpManual()
{
	return _joyMsg.axes[_controlIndices->AXIS_LINEAR_X] * _attitudeScales->LINEAR_X ;
}

double joy_control::JoyControl::getYawSpManual()
{
	return _joyMsg.axes[_controlIndices->AXIS_ANGULAR_YAW] * _attitudeScales->ANGULAR_Z ;
}

double joy_control::JoyControl::getThrustSpUnscaled()
{
	return (_joyMsg.axes[_controlIndices->AXIS_LINEAR_Z] + 1) / 2.0;
}

double joy_control::JoyControl::getYOffsetManual()
{
	return _joyMsg.axes[_controlIndices->AXIS_LINEAR_Y] * _positionScales->LINEAR_Y;
}

double joy_control::JoyControl::getXOffsetManual()
{
	return _joyMsg.axes[_controlIndices->AXIS_LINEAR_X]  * _positionScales->LINEAR_X;
}

double joy_control::JoyControl::getZOffsetManual()
{
	return _joyMsg.axes[_controlIndices->AXIS_LINEAR_Z] * _positionScales->LINEAR_Z;
}

double joy_control::JoyControl::getYawScale()
{
	return _attitudeScales->ANGULAR_Z;
}

double joy_control::JoyControl::getThrustScale()
{
	return _attitudeScales->LINEAR_Z;
}

void joy_control::JoyControl::initializeParameters(ros::NodeHandle& nh)
{
    ROS_WARN("JoyControl::initializeParameters()");
	
	// Load all control inputs indices
	bool initialized = 
		nh.getParam("/joy/axis_linear/x", 			_controlIndices->AXIS_LINEAR_X) &&
		nh.getParam("/joy/axis_linear/y", 			_controlIndices->AXIS_LINEAR_Y) &&
		nh.getParam("/joy/axis_linear/z", 			_controlIndices->AXIS_LINEAR_Z) &&
		nh.getParam("/joy/axis_angular/yaw", 		_controlIndices->AXIS_ANGULAR_YAW);
	ROS_INFO_STREAM(*_controlIndices);
	if (!initialized)
	{
		ROS_FATAL("JoyControl::initializeParameters() - \ 
			ControlIndices parameters are not properly set.");
		throw std::runtime_error("ControlIndices parameters not properly set.");
	}

	// Load all attitude scales
	initialized = 
		nh.getParam("/joy/scale_attitude/x", 		_attitudeScales->LINEAR_X) &&
		nh.getParam("/joy/scale_attitude/y", 		_attitudeScales->LINEAR_Y) &&
		nh.getParam("/joy/scale_attitude/z", 		_attitudeScales->LINEAR_Z) &&
		nh.getParam("/joy/scale_attitude/yaw", 		_attitudeScales->ANGULAR_Z);
	ROS_INFO_STREAM("Attitude " << *_attitudeScales);
	if (!initialized)
	{
		ROS_FATAL("JoyControl::initializeParameters() - \
			Attitude weight parameters are not properly set.");
		throw std::runtime_error("Attitude weight parameters are not properly set.");
	}

	// Load all position scales
	_positionScales->ANGULAR_Z = 0;
	initialized = 
		nh.getParam("/joy/scale_position/x", 		_positionScales->LINEAR_X) &&
		nh.getParam("/joy/scale_position/y", 		_positionScales->LINEAR_Y) &&
		nh.getParam("/joy/scale_position/z", 		_positionScales->LINEAR_Z);	
	ROS_INFO_STREAM("Position " << *_positionScales);
	if (!initialized)
	{
		ROS_FATAL("JoyControl::initializeParameters() - \
			Position weight parameters are not properly set.");
		throw std::runtime_error("Position weight parameters are not properly set.");
	}
}

const std::vector<int32_t> joy_control::JoyControl::getJoyButtons()
{
	return _joyMsg.buttons;
}

const std::vector<float> joy_control::JoyControl::getJoyAxes()
{
	return _joyMsg.axes;
}