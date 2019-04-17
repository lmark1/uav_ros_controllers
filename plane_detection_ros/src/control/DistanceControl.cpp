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
#include <mav_msgs/RollPitchYawrateThrust.h>

#include <array>

DistanceControl::DistanceControl(DistanceControlMode mode) :
	_mode(mode),
	_currState(DistanceControlState::MANUAL),
	_deactivateInspection(false),
	_distRef(-1),
	ControlBase()
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
		deactivateInspection();
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
		deactivateInspection();
	}
}

void DistanceControl::deactivateInspection()
{
	_currState = DistanceControlState::MANUAL;
	_deactivateInspection = true;
	getPID().resetIntegrator();
	ROS_WARN("Inspection mode deactivated successfully.");
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
	ROS_DEBUG("Calculating setpoint...");
	if (inInspectionState())
	{
		_attitudeSetpoint[0] = - getRollSpManual();
		_attitudeSetpoint[1] = - getPID().compute(_distRef, getDistanceMeasured(), dt);
		_attitudeSetpoint[2] = getUAVYaw() + getPlaneYaw();
	}
	else
	{
		_attitudeSetpoint[0] = - getRollSpManual();
		_attitudeSetpoint[1] = getPitchSpManual();
		_attitudeSetpoint[2] = getYawSpManual();
	}
}

void DistanceControl::publishDistanceSetpoint(ros::Publisher& pub)
{
	std_msgs::Float64 newMessage;
	newMessage.data = _distRef;
	pub.publish(newMessage);
}

void DistanceControl::publishSetpoint(ros::Publisher& pub)
{	
	// If in REAL mode, publish mavros::msgs AttitudeTarget
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
		newMessage.thrust = getThrustSpUnscaled();
		pub.publish(newMessage);
		return;
	}

	// If in simulation mode, publish mav_msgs::RollPitchYawrateThrust
	if (_mode == DistanceControlMode::SIMULATION)
	{
		mav_msgs::RollPitchYawrateThrust newMessage;
		newMessage.roll = _attitudeSetpoint[0];
		newMessage.pitch = _attitudeSetpoint[1];
		newMessage.yaw_rate = _attitudeSetpoint[2];
		newMessage.thrust = geometry_msgs::Vector3();
		newMessage.thrust.x = 0;
		newMessage.thrust.y = 0;
		newMessage.thrust.z = getThrustSpManual();
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
	return !inspectionEnabledJoy() &&
			_currState == DistanceControlState::INSPECTION;
}
