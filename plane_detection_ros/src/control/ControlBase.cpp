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
	_distanceVelPID (new PID),
	_posYPID (new PID),
	_posZPID (new PID),
	_joyIndices (new joy_control::JoyIndices),
	_joyScales (new joy_control::ScaleWeights)
{
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
	
	updatePosition(
		message->pose.pose.position.x,
		message->pose.pose.position.y,
		message->pose.pose.position.z);
}

void ControlBase::posCbReal(const geometry_msgs::PoseStampedConstPtr& message)
{
	_currentPosition[0] = message->pose.position.x;
	_currentPosition[1] = message->pose.position.y;
	_currentPosition[2] = message->pose.position.z;

	updatePosition(
		message->pose.position.x,
		message->pose.position.y,
		message->pose.position.z
	);
}

void ControlBase::updatePosition(double x, double y, double z)
{
	_currentPosition[0] = x * cos(_uavYaw) - y * sin(_uavYaw);
	_currentPosition[1] = x * sin(_uavYaw) + y * cos(_uavYaw);
	_currentPosition[2] = z;
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

PID& ControlBase::getPitchPID()
{
	return *_distancePID;
}

PID& ControlBase::getPitchRatePID()
{
	return *_distanceVelPID;
}

PID& ControlBase::getPosYPID()
{
	return *_posYPID;
}

PID& ControlBase::getPosZPID()
{
	return *_posZPID;
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

const std::array<double, 3>& ControlBase::getCurrPosition()
{
	return _currentPosition;
}

void ControlBase::parametersCallback(
		plane_detection_ros::DistanceControlParametersConfig& configMsg,
		uint32_t level)
{
	ROS_DEBUG("ControlBase::parametersCallback()");
	_distancePID->set_kp(configMsg.k_p_x);
	_distancePID->set_kd(configMsg.k_d_x);
	_distancePID->set_ki(configMsg.k_i_x);
	_distancePID->set_lim_high(configMsg.lim_high_x);
	_distancePID->set_lim_low(configMsg.lim_low_x);

	_distanceVelPID->set_kp(configMsg.k_p_vx);
	_distanceVelPID->set_kd(configMsg.k_d_vx);
	_distanceVelPID->set_ki(configMsg.k_i_vx);
	_distanceVelPID->set_lim_high(configMsg.lim_high_vx);
	_distanceVelPID->set_lim_low(configMsg.lim_low_vx);

	_posYPID->set_kp(configMsg.k_p_y);
	_posYPID->set_kd(configMsg.k_d_y);
	_posYPID->set_ki(configMsg.k_i_y);
	_posYPID->set_lim_high(configMsg.lim_high_y);
	_posYPID->set_lim_low(configMsg.lim_low_y);

	_posZPID->set_kp(configMsg.k_p_z);
	_posZPID->set_kd(configMsg.k_d_z);
	_posZPID->set_ki(configMsg.k_i_z);
	_posZPID->set_lim_high(configMsg.lim_high_z);
	_posZPID->set_lim_low(configMsg.lim_low_z);
}

void ControlBase::initializeParameters(ros::NodeHandle& nh)
{
	ROS_DEBUG("ControlBase::initializeParameters()");
	
	// Try to get all joy_indices
	bool indicesRead = 
		nh.getParam("/control/axis_linear/x", 			_joyIndices->AXIS_LINEAR_X) &&
		nh.getParam("/control/axis_linear/y", 			_joyIndices->AXIS_LINEAR_Y) &&
		nh.getParam("/control/axis_linear/z", 			_joyIndices->AXIS_LINEAR_Z) &&
		nh.getParam("/control/axis_angular/yaw", 		_joyIndices->AXIS_ANGULAR_YAW) &&
		nh.getParam("/control/detection_state", 		_joyIndices->INSPECTION_MODE); 
	ROS_INFO_STREAM(*_joyIndices);
	if (! indicesRead)
	{
		ROS_FATAL("ControlBase::initializeParameters()- JoyIndices parameters are not properly set.");
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
		ROS_FATAL("ControlBase::initializeParameters() - JoyScales parameters are not properly set.");
		throw std::invalid_argument("JoyScales parameters are not properly set.");
	}

	bool initialized =
		nh.getParam("/control/pitch/kp", _distancePID->get_kp_ref()) &&
		nh.getParam("/control/pitch/ki", _distancePID->get_ki_ref()) &&
		nh.getParam("/control/pitch/kd", _distancePID->get_kd_ref()) &&
		nh.getParam("/control/pitch/lim_low", _distancePID->get_lim_low_ref()) &&
		nh.getParam("/control/pitch/lim_high", _distancePID->get_lim_high_ref());
	ROS_INFO_STREAM(*_distancePID);
	if (!initialized)
	{
		ROS_FATAL("ControlBase::initializeParameters() - parameter initialization failed.");
		throw std::invalid_argument("Pitch PID parameters not properly set.");
	}

	initialized = 
		nh.getParam("/control/pitch_rate/kp", _distanceVelPID->get_kp_ref()) &&
		nh.getParam("/control/pitch_rate/ki", _distanceVelPID->get_ki_ref()) &&
		nh.getParam("/control/pitch_rate/kd", _distanceVelPID->get_kd_ref()) &&
		nh.getParam("/control/pitch_rate/lim_low", _distanceVelPID->get_lim_low_ref()) &&
		nh.getParam("/control/pitch_rate/lim_high", _distanceVelPID->get_lim_high_ref());
	ROS_INFO_STREAM(*_distanceVelPID);
	if (!initialized)
	{
		ROS_FATAL("ControlBase::initializeParameters() - parameter initialization failed.");
		throw std::invalid_argument("Pitch rate PID parameters not properly set.");
	}

	initialized = 
		nh.getParam("/control/pos_y/kp", _posYPID->get_kp_ref()) &&
		nh.getParam("/control/pos_y/ki", _posYPID->get_ki_ref()) &&
		nh.getParam("/control/pos_y/kd", _posYPID->get_kd_ref()) &&
		nh.getParam("/control/pos_y/lim_low", _posYPID->get_lim_low_ref()) &&
		nh.getParam("/control/pos_y/lim_high", _posYPID->get_lim_high_ref());
	ROS_INFO_STREAM(*_posYPID);
	if (!initialized)
	{
		ROS_FATAL("ControlBase::initializeParameters() - parameter initialization failed.");
		throw std::invalid_argument("Y-pos PID parameters not properly set.");
	}

	initialized = 
		nh.getParam("/control/pos_z/kp", _posZPID->get_kp_ref()) &&
		nh.getParam("/control/pos_z/ki", _posZPID->get_ki_ref()) &&
		nh.getParam("/control/pos_z/kd", _posZPID->get_kd_ref()) &&
		nh.getParam("/control/pos_z/lim_low", _posZPID->get_lim_low_ref()) &&
		nh.getParam("/control/pos_z/lim_high", _posZPID->get_lim_high_ref());
	ROS_INFO_STREAM(*_posZPID);
	if (!initialized)
	{
		ROS_FATAL("ControlBase::initializeParameters() - parameter initialization failed.");
		throw std::invalid_argument("Z-pos PID parameters not properly set.");
	}
}

void ControlBase::setReconfigureParameters(plane_detection_ros::DistanceControlParametersConfig& config)
{
	ROS_WARN("ControlBase::setreconfigureParameters()");
	config.k_p_x = _distancePID->get_kp();
	config.k_i_x = _distancePID->get_ki();
	config.k_d_x = _distancePID->get_kd();
	config.lim_low_x = _distancePID->get_lim_low();
	config.lim_high_x = _distancePID->get_lim_high();
	
	config.k_p_vx = _distanceVelPID->get_kp();
	config.k_i_vx = _distanceVelPID->get_ki();
	config.k_d_vx = _distanceVelPID->get_kd();
	config.lim_low_vx = _distanceVelPID->get_lim_low();
	config.lim_high_vx = _distanceVelPID->get_lim_high();

	config.k_p_y = _posYPID->get_kp();
	config.k_i_y = _posYPID->get_ki();
	config.k_d_y = _posYPID->get_kd();
	config.lim_low_y = _posYPID->get_lim_low();
	config.lim_high_y = _posYPID->get_lim_high();
	
	config.k_p_z = _posZPID->get_kp();
	config.k_i_z = _posZPID->get_ki();
	config.k_d_z = _posZPID->get_kd();
	config.lim_low_z = _posZPID->get_lim_low();
	config.lim_high_z = _posZPID->get_lim_high();
}