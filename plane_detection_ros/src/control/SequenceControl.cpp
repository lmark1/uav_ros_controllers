#include <ros/ros.h>
#include <plane_detection_ros/control/SequenceControl.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#define SEQ_STEP_PARAM "/sequence/step"
#define TRIGGER_THRESHOLD 0.5

SequenceControl::SequenceControl(bool simMode):
	_sequenceStep (0),	
	_distToCarrot (-1),
	_inSequence (false)
{
}

SequenceControl::~SequenceControl()
{
}


void SequenceControl::carrotDistCb(const std_msgs::Float64ConstPtr& message)
{
	_distToCarrot = message->data;
}

void SequenceControl::sequenceCb(const std_msgs::BoolConstPtr& message)
{
	_inSequence = message->data;
}

void SequenceControl::publishSequenceOffset(ros::Publisher& pub)
{
	std_msgs::Float64 newMessage;
	if (_inSequence)
		newMessage.data = _sequenceStep;
	else
		newMessage.data = 0;
	pub.publish(newMessage);
}

void SequenceControl::publishSequenceOffset(ros::Publisher& pub, double offset)
{
	std_msgs::Float64 newMessage;
	newMessage.data = offset;
	pub.publish(newMessage);
}

bool SequenceControl::sequenceActive()
{
	return _inSequence;
}

double SequenceControl::getSequenceStep()
{
	return _sequenceStep;
}

double SequenceControl::getDistanceToCarrot()
{
	return _distToCarrot;
}

void SequenceControl::initializeParameters(ros::NodeHandle& nh)
{
	bool initialized = nh.getParam(SEQ_STEP_PARAM, _sequenceStep);
	ROS_INFO("New sequence step set: %.2f", _sequenceStep);
	if (!initialized)
	{
		ROS_FATAL("DistanceControl::initializeParameters() - sequence step not properly set.");
		throw std::runtime_error("DistanceControl parameters are not properly set.");
	}
}