/*
 * DistanceControl.cpp
 *
 *  Created on: Apr 11, 2019
 *      Author: lmark
 */

#include "plane_detection_ros/control/ControlBase.h"

// Cpp includes
#include <iostream>
#include <vector>
#include <array>
#include <math.h>

/**
 * Calculate yaw angle from given quaternion parameters.
 *
 */
double calculateYaw(double qx, double qy, double qz, double qw)
{
	return atan2(
			2 * (qw * qz + qx * qy),
			qw * qw + qx * qx - qy * qy - qz * qz);
}

ControlBase::ControlBase():
	_distanceMeasured (-1),
	_distanceVelocityMeasured (0),
	_distancePID (new PID),
	_distanceRatePID (new PID),
	_joyIndices (new joy_control::JoyIndices),
	_joyScales (new joy_control::ScaleWeights)
{
	// Make a node handle
	ros::NodeHandle nh;

	// Try to get all joy_indices
	bool indicesRead = 
		nh.getParam("/control/axis_linear/x", 			_joyIndices->AXIS_LINEAR_X) &&
		nh.getParam("/control/axis_linear/y", 			_joyIndices->AXIS_LINEAR_Y) &&
		nh.getParam("/control/axis_linear/z", 			_joyIndices->AXIS_LINEAR_Z) &&
		nh.getParam("/control/axis_angular/yaw", 		_joyIndices->AXIS_ANGULAR_YAW) &&
		nh.getParam("/control/detection_state", _joyIndices->INSPECTION_MODE); 
	ROS_INFO_STREAM(*_joyIndices);
	if (! indicesRead)
	{
		ROS_FATAL("ControlBase() - JoyIndeces parameters are not properly set.");
		throw std::invalid_argument("JoyIndices parameters not properly set.");
	}

	// Try to get all control input scales
	bool scalesRead = 
		nh.getParam("/control/scale_linear/x", 		_joyScales->LINEAR_X) &&
		nh.getParam("/control/scale_linear/y", 		_joyScales->LINEAR_Y) &&
		nh.getParam("/control/scale_linear/z", 		_joyScales->LINEAR_Z) &&
		nh.getParam("/control/scale_angular/yaw", 	_joyScales->ANGULAR_Z);
	ROS_INFO_STREAM(*_joyScales);
	if (! scalesRead)
	{
		ROS_FATAL("ControlBase() - JoyScales parameters are not properly set.");
		throw std::invalid_argument("JoyScales parameters are not properly set.");
	}

	// Initialize JoyMsg
	_joyMsg.axes = std::vector<float> (10, 0.0);
	_joyMsg.buttons = std::vector<int> (10, 0);
}

ControlBase::~ControlBase()
{
}

void ControlBase::distanceCb(const std_msgs::Float64ConstPtr& message)
{
	_distanceMeasured = message->data;
}

void ControlBase::distanceVelCb(const std_msgs::Float64ConstPtr& message)
{
	_distanceVelocityMeasured = message->data;
}

void ControlBase::joyCb(const sensor_msgs::JoyConstPtr& message)
{
	_joyMsg = *message;

	// Scale thrust joy input from -1 - 1 to 0-1
	_joyMsg.axes[_joyIndices->AXIS_LINEAR_Z] += 1;
	_joyMsg.axes[_joyIndices->AXIS_LINEAR_Z] /= 2;
}

void ControlBase::imuCbReal(const sensor_msgs::ImuConstPtr& message)
{
	_uavYaw = calculateYaw(
			message->orientation.x,
			message->orientation.y,
			message->orientation.z,
			message->orientation.w);
}

void ControlBase::imuCbSim(const nav_msgs::OdometryConstPtr& message)
{
	_uavYaw = calculateYaw(
			message->pose.pose.orientation.x,
			message->pose.pose.orientation.y,
			message->pose.pose.orientation.z,
			message->pose.pose.orientation.w);
}

void ControlBase::normalCb(const geometry_msgs::PoseStampedConstPtr& message)
{
	_planeYaw = calculateYaw(
			message->pose.orientation.x,
			message->pose.orientation.y,
			message->pose.orientation.z,
			message->pose.orientation.w);

	// Check in which direction is plane normal facing
	double xComponent = cos(_planeYaw);
	if (xComponent < 0)
		_planeYaw += M_PI;
}

void ControlBase::parametersCallback(
		plane_detection_ros::DistanceControlParametersConfig& configMsg,
		uint32_t level)
{
	_distancePID->set_kp(configMsg.k_p_x);
	_distancePID->set_kd(configMsg.k_d_x);
	_distancePID->set_ki(configMsg.k_i_x);
	_distancePID->set_lim_high(configMsg.lim_high_x);
	_distancePID->set_lim_low(configMsg.lim_low_x);

	_distanceRatePID->set_kp(configMsg.k_p_vx);
	_distanceRatePID->set_kd(configMsg.k_d_vx);
	_distanceRatePID->set_ki(configMsg.k_i_vx);
	_distanceRatePID->set_lim_high(configMsg.lim_high_vx);
	_distanceRatePID->set_lim_low(configMsg.lim_low_vx);
}

bool ControlBase::inspectionEnabledJoy()
{
	return _joyMsg.buttons[_joyIndices->INSPECTION_MODE] == 1;
}

double ControlBase::getDistanceMeasured()
{
	return _distanceMeasured;
}

double ControlBase::getDistanceVelMeasured()
{
	return _distanceVelocityMeasured;
}

PID& ControlBase::getPID()
{
	return *_distancePID;
}

PID& ControlBase::getPID_vx()
{
	return *_distanceRatePID;
}

double ControlBase::getPlaneYaw()
{
	return _planeYaw;
}

double ControlBase::getUAVYaw()
{
	return _uavYaw;
}

double ControlBase::getRollSpManual()
{
	return _joyMsg.axes[_joyIndices->AXIS_LINEAR_Y] * _joyScales->LINEAR_Y;
}

double ControlBase::getPitchSpManual()
{
	return _joyMsg.axes[_joyIndices->AXIS_LINEAR_X] * _joyScales->LINEAR_X;
}

double ControlBase::getYawSpManual()
{
	return _joyMsg.axes[_joyIndices->AXIS_ANGULAR_YAW] * _joyScales->ANGULAR_Z;
}

double ControlBase::getThrustSpManual()
{
	return _joyMsg.axes[_joyIndices->AXIS_LINEAR_Z] * _joyScales->LINEAR_Z;
}

double ControlBase::getThrustSpUnscaled()
{
	return _joyMsg.axes[_joyIndices->AXIS_LINEAR_Z];
}

double ControlBase::getYawScale()
{
	return _joyScales->ANGULAR_Z;
}
