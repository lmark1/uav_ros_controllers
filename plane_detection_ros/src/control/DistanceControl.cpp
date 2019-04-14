/*
 * DistanceControl.cpp
 *
 *  Created on: Apr 13, 2019
 *      Author: lmark
 */

#include "control/DistanceControl.h"
#include <ros/ros.h>
#include <std_msgs/Int32.h>

DistanceControl::DistanceControl(DistanceControlMode mode):
	_mode(mode),
	_currState(DistanceControlState::MANUAL),
	_deactivateInspection(false),
	_distRef(-1)
{
	// TODO Auto-generated constructor stub

}

DistanceControl::~DistanceControl() {
	// TODO Auto-generated destructor stub
}

void DistanceControl::detectStateChange()
{
	// If we're in inspection mode and received distance is invalid
	// then deactivate inspection mode !
	if (inspectionFailed())
	{
		ROS_FATAL("Inspection mode failed - invalid distance");
		_deactivateInspection = true;
		_currState = DistanceControlState::MANUAL;
		return;
	}

	// Check if user wants to go to inspection mode
	if (inspectionRequested())
	{
		ROS_DEBUG("Inspection mode requested.");
		// Check if current distance is valid
		if (_distance < 0)
		{
			ROS_FATAL("Unable to enter inspection mode.");
			return;
		}

		ROS_INFO("Inspection activation successful-following distance %.2f",
				_distance);
		_distRef = _distance;
		_currState = DistanceControlState::INSPECTION;
		return;
	}

	// Check if user wants to go back to manual mode
	if (manualRequested())
	{
		ROS_INFO("Manual mode entered.");
		_currState = DistanceControlState::MANUAL;
		_deactivateInspection = true;
	}
}

void DistanceControl::publishState(ros::Publisher& pub)
{
	// Publish 0 for manual state and 1 for inspection state
	std_msgs::Int32 newMessage;
	newMessage.data = _currState == DistanceControlState::MANUAL ? 0 : 1;
	pub.publish(newMessage);
}

bool DistanceControl::inspectionRequested()
{
	return _joyMsg.buttons[4] == 1 &&
			_currState == DistanceControlState::MANUAL;
}

bool DistanceControl::inspectionFailed()
{
	return _currState == DistanceControlState::INSPECTION &&
			_distance < 0;
}

bool DistanceControl::manualRequested()
{
	return _joyMsg.buttons[4] != 1 &&
			_currState == DistanceControlState::INSPECTION;
}
