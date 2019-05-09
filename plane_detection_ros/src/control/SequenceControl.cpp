#include <ros/ros.h>
#include <plane_detection_ros/control/SequenceControl.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#define DETECTION_STATE_PARAM "/joy/detection_state"
#define RIGHT_SEQ_PARAM "/joy/left_seq"
#define LEFT_SEQ_PARAM "/joy/right_seq"
#define SEQ_STEP_PARAM "/sequence/step"
#define TRIGGER_THRESHOLD 0.5

SequenceControl::SequenceControl(bool simMode):
    _inspectIndices {new joy_struct::InspectionIndices},
	_sequenceStep (0.5),
	_simMode (simMode),
	_inspectionEnabled (false),
	_rightSequenceEnabled (false),
	_leftSequenceEnabled (false)
{
}

SequenceControl::~SequenceControl()
{
}

void SequenceControl::joyCb(const sensor_msgs::JoyConstPtr& message)
{
	if (_simMode)
	{
		_inspectionEnabled = message->buttons[_inspectIndices->INSPECTION_MODE] == 1;
		_leftSequenceEnabled = message->buttons[_inspectIndices->LEFT_SEQUENCE] == 1;
		_rightSequenceEnabled = message->buttons[_inspectIndices->RIGHT_SEQUENCE] == 1;
	}
	else
	{
		_inspectionEnabled = message->buttons[_inspectIndices->INSPECTION_MODE] == 0;
		_leftSequenceEnabled = message->axes[_inspectIndices->RIGHT_SEQUENCE] > TRIGGER_THRESHOLD;
		_rightSequenceEnabled = message->axes[_inspectIndices->RIGHT_SEQUENCE] < TRIGGER_THRESHOLD;
	}
}

void SequenceControl::publishSequenceOffset(ros::Publisher& pub)
{
	std_msgs::Float64 newMessage;

	newMessage.data = 0;
	if (_leftSequenceEnabled)
		newMessage.data = _sequenceStep;
	else if (_rightSequenceEnabled)
		newMessage.data = - _sequenceStep;
	
	pub.publish(newMessage);
}

void SequenceControl::publishInpsectionMode(ros::Publisher& pub)
{
	std_msgs::Bool newMessage;
	newMessage.data = _inspectionEnabled;
	pub.publish(newMessage);
}

void SequenceControl::publishLeftSequence(ros::Publisher& pub)
{
	std_msgs::Bool newMessage;
	newMessage.data = _leftSequenceEnabled;
	pub.publish(newMessage);
}

void SequenceControl::publishRightSequence(ros::Publisher& pub)
{
	std_msgs::Bool newMessage;
	newMessage.data = _rightSequenceEnabled;
	pub.publish(newMessage);
}

void SequenceControl::initializeParameters(ros::NodeHandle& nh)
{
	bool initialized = 
		nh.getParam(DETECTION_STATE_PARAM, _inspectIndices->INSPECTION_MODE) &&
		nh.getParam(LEFT_SEQ_PARAM, _inspectIndices->LEFT_SEQUENCE) &&
		nh.getParam(RIGHT_SEQ_PARAM, _inspectIndices->RIGHT_SEQUENCE);
	ROS_INFO_STREAM(*_inspectIndices);
	if (!initialized)
	{
		ROS_FATAL("DistanceControl::initializeParameters() - inspection index not set.");
		throw std::runtime_error("DistanceControl parameters are not properly set.");
	}

	initialized = nh.getParam(SEQ_STEP_PARAM, _sequenceStep);
	ROS_INFO("New sequence step set: %.2f", _sequenceStep);
	if (!initialized)
	{
		ROS_FATAL("DistanceControl::initializeParameters() - sequence step not properly set.");
		throw std::runtime_error("DistanceControl parameters are not properly set.");
	}
}