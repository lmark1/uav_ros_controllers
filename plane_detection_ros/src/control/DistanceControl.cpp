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
	_distVelSp(0),
	_distSp(-1),
	_hoverThrust (0),
	_deactivateInspection(false),
	_inspectionRequestFailed(false),
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
	if (inInspectionState() && fabs(getDistanceMeasured() - _distSp) > 2)
	{
		ROS_FATAL("Deactivating inspection mode - outside tolerance");
		deactivateInspection();
		return;
	}

	// If we're in inspection mode and received distance is invalid
	// then deactivate inspection mode !
	if (inspectionFailed())
	{
		ROS_FATAL("Inspection mode failed - invalid distance");
		deactivateInspection();
		_inspectionRequestFailed = true;
		return;
	}

	// Check if user wants to go to inspection mode
	if (inspectionRequested() && !_inspectionRequestFailed)
	{
		ROS_DEBUG("Inspection mode requested.");
		// Check if current distance is valid
		if (getDistanceMeasured() < 0)
		{
			ROS_FATAL("Unable to enter inspection mode.");
			_inspectionRequestFailed = true;
			return;
		}

		ROS_INFO("Inspection activation successful-following distance %.2f",
				getDistanceMeasured());
		_distSp = getDistanceMeasured();
		_currState = DistanceControlState::INSPECTION;
		_carrotPos[0] = getCurrPosition()[0];
		_carrotPos[1] = getCurrPosition()[1];
		_carrotPos[2] = getCurrPosition()[2];
		return;
	}

	// Check if user wants to go back to manual mode
	if (manualRequested())
	{
		ROS_INFO("Manual mode entered.");
		_inspectionRequestFailed = false;
		deactivateInspection();
	}
}

void DistanceControl::deactivateInspection()
{
	_currState = DistanceControlState::MANUAL;
	_deactivateInspection = true;
	
	// Reset all PIDs
	getDistancePID().resetIntegrator();
	getDistanceVelPID().resetIntegrator();
	getPosYPID().resetIntegrator();
	getVelYPID().resetIntegrator();
	getPosZPID().resetIntegrator();
	getVelZPID().resetIntegrator();
	
	// Reset Carrot position
	_carrotPos[0] = getCurrPosition()[0];
	_carrotPos[1] = getCurrPosition()[1];
	_carrotPos[2] = getCurrPosition()[2];
	ROS_WARN("Inspection mode deactivated successfully.");
}

void DistanceControl::publishState(ros::Publisher& pub)
{
	// Publish 0 for manual state and 1 for inspection state
	std_msgs::Int32 newMessage;
	newMessage.data = inInspectionState() ? 1 : 0;
	pub.publish(newMessage);
}

void DistanceControl::publishDistVelSp(ros::Publisher& pub)
{
	std_msgs::Float64 newMessage;
	newMessage.data = _distVelSp;
	pub.publish(newMessage);
}

void DistanceControl::calculateAttitudeTarget(double dt)
{	
	// Calculate setpoint outside inspection state
	if (!inInspectionState())
	{	
		_attThrustSp[0] = - getRollSpManual();
		_attThrustSp[1] = getPitchSpManual();
		_attThrustSp[2] = - getYawSpManual();	
		_attThrustSp[3] = getThrustSpUnscaled();
		return;
	}
	
	// Calculate setpoint inside inspection state
	_attThrustSp[0] = - getRollSpManual();
	_distVelSp = getDistancePID().compute(_distSp, getDistanceMeasured(), dt);
	_attThrustSp[1] = - getDistanceVelPID().compute(_distVelSp, getDistanceVelMeasured(), dt);

	// If in simulation mode treat as YAW RATE, otherwise treat as YAW
	if (_mode == DistanceControlMode::SIMULATION)
		_attThrustSp[2] = getPlaneYaw() * 10;
	else
		_attThrustSp[2] = getUAVYaw() - getPlaneYaw();

	// Set thrust
	_attThrustSp[3] = getThrustSpUnscaled();
}

void DistanceControl::calculateCarrotSetpoint(double dt)
{
	// Update carrot
	_carrotPos[0] += getPitchSpManual();
	_carrotPos[1] += getRollSpManual();
	_carrotPos[2] += getZPosSpManual();
	ROS_DEBUG("Carrot pos: [%.2f, %.2f, %.2f]", _carrotPos[0], _carrotPos[1], _carrotPos[2]);
	ROS_DEBUG("Current pos: [%.2f, %.2f, %.2f]", getCurrPosition()[0], getCurrPosition()[1], getCurrPosition()[2]);
	ROS_DEBUG("Current vel: [%.2f, %.2f, %.2f]", getCurrVelocity()[0], getCurrVelocity()[1], getCurrVelocity()[2]);
	
	// Always the same along the y, z axes
	double velSpY = getPosYPID().compute(_carrotPos[1], getCurrPosition()[1], dt);
	double velSpZ = getPosZPID().compute(_carrotPos[2], getCurrPosition()[2], dt);
	_attThrustSp[0] = - getVelYPID().compute(velSpY, getCurrVelocity()[1], dt);
	_attThrustSp[3] = getVelZPID().compute(velSpZ, getCurrVelocity()[2], dt);
	ROS_DEBUG("VelSpZ=%.2f\tThrust=%.2f", velSpZ, _attThrustSp[3]);

	// Carrot tracking if not in inspection mode
	if (!inInspectionState())
	{
		double velSpX = getPosXPID().compute(_carrotPos[0], getCurrPosition()[0], dt);
		_attThrustSp[1] = getVelXPID().compute(velSpX, getCurrVelocity()[0], dt);
		_attThrustSp[2] = - getYawSpManual();
		return;
	}

	// Calculate pitch setpoint using measured distance
	_distVelSp = getDistancePID().compute(_distSp, getDistanceMeasured(), dt);
	_attThrustSp[1] = - getDistanceVelPID().compute(_distVelSp, getDistanceVelMeasured(), dt);

	// If in simulation mode treat as YAW RATE, otherwise treat as YAW
	if (_mode == DistanceControlMode::SIMULATION)
		_attThrustSp[2] = getPlaneYaw() * 10;
	else
		_attThrustSp[2] = getUAVYaw() - getPlaneYaw();
}

void DistanceControl::publishDistSp(ros::Publisher& pub)
{
	std_msgs::Float64 newMessage;
	newMessage.data = _distSp;
	pub.publish(newMessage);
}

void DistanceControl::publishAttSp(ros::Publisher& pub)
{	
	mavros_msgs::AttitudeTarget newMessage;
	tf2::Quaternion myQuaternion;

	// If in REAL mode, publish mavros::msgs AttitudeTarget
	if (_mode == DistanceControlMode::REAL)
	{
		if (inInspectionState())
		{
			// Yaw is controlled in radians
			myQuaternion.setEulerZYX(
				_attThrustSp[2],
				_attThrustSp[1],
				_attThrustSp[0]);

			//Ignore roll rate, pitch rate, yaw rate.
			newMessage.type_mask = 7; 
		}
		else 
		{
			// Yaw is controlled using yaw rate
			myQuaternion.setEulerZYX(
				0,
				_attThrustSp[1],
				_attThrustSp[0]);
			
			//Ignore roll rate, pitch rate
			newMessage.type_mask = 3; 
			newMessage.body_rate.z = _attThrustSp[2];
		}

		newMessage.orientation.x = myQuaternion.x();
		newMessage.orientation.y = myQuaternion.y();
		newMessage.orientation.z = myQuaternion.z();
		newMessage.orientation.w = myQuaternion.w();	
		newMessage.thrust = _attThrustSp[3] + _hoverThrust;
		ROS_DEBUG("MavrosMsg - thrust = %.2f", newMessage.thrust);
		pub.publish(newMessage);
		return;
	}

	// If in simulation mode, publish mav_msgs::RollPitchYawrateThrust
	if (_mode == DistanceControlMode::SIMULATION)
	{
		mav_msgs::RollPitchYawrateThrust newMessage;
		newMessage.roll = _attThrustSp[0];
		newMessage.pitch = _attThrustSp[1];
		newMessage.yaw_rate = _attThrustSp[2];
		newMessage.thrust = geometry_msgs::Vector3();
		newMessage.thrust.x = 0;
		newMessage.thrust.y = 0;
		newMessage.thrust.z = _attThrustSp[3] * getThrustScale(); // Add base thrust
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
	return !inspectionEnabledJoy() && (
		_currState == DistanceControlState::INSPECTION || 
		_inspectionRequestFailed);
}

void DistanceControl::publishEulerSp(ros::Publisher& pub)
{
	geometry_msgs::Vector3 newMessage;
	newMessage.x = _attThrustSp[0];
	newMessage.y = _attThrustSp[1];
	newMessage.z = _attThrustSp[2];
	pub.publish(newMessage);
}

void DistanceControl::initializeParameters(ros::NodeHandle& nh)
{
	ControlBase::initializeParameters(nh);
	ROS_WARN("DistanceControl::initializeParameters()");
	bool initialized = nh.getParam("/control/hover", _hoverThrust);
	ROS_INFO("New hover thrust: %.2f", _hoverThrust);
	if (!initialized)
	{
		ROS_FATAL("DistanceControl::initalizeParameters() - failed to initialize hover thrust");
		throw std::invalid_argument("DistanceControl parameters not properly initialized.");
	}
}

void DistanceControl::parametersCallback(
		plane_detection_ros::DistanceControlParametersConfig& configMsg,
		uint32_t level)
{
	ControlBase::parametersCallback(configMsg, level);
	ROS_WARN("DistanceControl::parametersCallback()");;
	_hoverThrust = configMsg.hover;
	ROS_INFO("New hover thrust: %.2f", _hoverThrust);
}

void DistanceControl::setReconfigureParameters(plane_detection_ros::DistanceControlParametersConfig& config)
{
	ControlBase::setReconfigureParameters(config);
	ROS_WARN("DistanceControl::setreconfigureParameters()");
	config.hover = _hoverThrust;
}