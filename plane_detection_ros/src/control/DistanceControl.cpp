/*
 * DistanceControl.cpp
 *
 *  Created on: Apr 13, 2019
 *      Author: lmark
 */

#include "plane_detection_ros/control/DistanceControl.h"

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <tf2/LinearMath/Quaternion.h>


DistanceControl::DistanceControl(DistanceControlMode mode,
	double kp, double ki, double kd, double limLow, double limHigh) :
	_mode(mode),
	_currState(DistanceControlState::MANUAL),
	_deactivateInspection(false),
	_distRef(-1),
	ControlBase(kp, ki, kd, limLow, limHigh)
{

	// Info messages about node start.
	if (_mode == DistanceControlMode::SIMULATION)
		ROS_INFO("DistanceControl: Starting node in simulation mode.");
	else
		ROS_INFO("DistanceControl: Starting node in real mode.");
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
		if (getDistanceMeasured() < 0)
		{
			ROS_FATAL("Unable to enter inspection mode.");
			return;
		}

		ROS_INFO("Inspection activation successful-following distance %.2f",
				getDistanceMeasured());
		_distRef = getDistanceMeasured();
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
	newMessage.data = inInspectionState() ? 1 : 0;
	pub.publish(newMessage);
}

void DistanceControl::calculateSetpoint(double dt)
{
	// If not in inspection mode do not do anything.
	if (!inInspectionState())
		return;

	_attitudeSetpoint[0] = getPID()
			.compute(_distRef, getDistanceMeasured(), dt);
	_attitudeSetpoint[1] = getRollSetpoint();
	_attitudeSetpoint[2] = getYawSetpoint();
}

void DistanceControl::publishSetpoint(ros::Publisher& pub)
{
	if (_mode == DistanceControlMode::REAL)
	{
		tf2::Quaternion myQuaternion;
		myQuaternion.setEuler(
				_attitudeSetpoint[0],
				_attitudeSetpoint[1],
				_attitudeSetpoint[2]);

		mavros_msgs::AttitudeTarget newMessage;
		newMessage.type_mask = 7; //Ignore roll, pitch, yaw rate.
		newMessage.orientation.x = myQuaternion.x();
		newMessage.orientation.y = myQuaternion.y();
		newMessage.orientation.z = myQuaternion.z();
		newMessage.orientation.w = myQuaternion.w();
		pub.publish(newMessage);
		return;
	}

	if (_mode == DistanceControlMode::SIMULATION)
	{
		geometry_msgs::Vector3 newMessage;
		newMessage.x = _attitudeSetpoint[0];
		newMessage.y = _attitudeSetpoint[1];
		newMessage.z = _attitudeSetpoint[2];
		pub.publish(newMessage);
		return;
	}
}

bool DistanceControl::inInspectionState()
{
	return _currState == DistanceControlState::INSPECTION;
}

bool DistanceControl::inspectionRequested()
{
	return inspectionEnabledJoy() &&
			_currState == DistanceControlState::MANUAL;
}

bool DistanceControl::inspectionFailed()
{
	return _currState == DistanceControlState::INSPECTION &&
			getDistanceMeasured() < 0;
}

bool DistanceControl::manualRequested()
{
	return !inspectionEnabledJoy() != 1 &&
			_currState == DistanceControlState::INSPECTION;
}
