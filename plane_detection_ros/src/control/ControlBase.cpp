/*
 * DistanceControl.cpp
 *
 *  Created on: Apr 11, 2019
 *      Author: lmark
 */

#include "plane_detection_ros/control/ControlBase.h"
#include <nav_msgs/Odometry.h>

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
	_distancePID (new PID ("Distance")),
	_distanceVelPID (new PID ("DistanceVel")),
	_posYPID (new PID ("Y-Position")),
	_velYPID (new PID ("Y-Velocity")),
	_posXPID (new PID ("X-Position")),
	_velXPID (new PID ("X-Velocity")),
	_posZPID (new PID ("Z-Position")),
	_velZPID (new PID ("Z-Velocity")),
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

	rotateVector(
		message->pose.pose.position.x,
		message->pose.pose.position.y,
		message->pose.pose.position.z,
		_currentPosition);

	_currentVelocity[0] = message->twist.twist.linear.x;
	_currentVelocity[1] = message->twist.twist.linear.y;
	_currentVelocity[2] = message->twist.twist.linear.z;
	// Velocity from Odometry is in local coordinate system
}

void ControlBase::posCbReal(const nav_msgs::OdometryConstPtr& message)
{
	rotateVector(
		message->pose.pose.position.x,
		message->pose.pose.position.y,
		message->pose.pose.position.z,
		_currentPosition);
}

void ControlBase::velCbReal(const nav_msgs::OdometryConstPtr& message)
{
	rotateVector(
		message->twist.twist.linear.x,
		message->twist.twist.linear.y,
		- message->twist.twist.linear.z,
		_currentVelocity);
}

void ControlBase::rotateVector(
	const double x, const double y, const double z, std::array<double, 3>& vector)
{
	vector[0] = x * cos(-_uavYaw) - y * sin(-_uavYaw);
	vector[1] = x * sin(-_uavYaw) + y * cos(-_uavYaw);
	vector[2] = z;
}


/* wrap x -> [0,max) */
double wrapMax(double x, double max)
{
    /* integer math: `(max + x % max) % max` */
    return fmod(max + fmod(x, max), max);
}
/* wrap x -> [min,max) */
double wrapMinMax(double x, double min, double max)
{
    return min + wrapMax(x - min, max - min);
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

	_planeYaw = wrapMinMax(_planeYaw, -M_PI, M_PI);
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

PID& ControlBase::getDistancePID()
{
	return *_distancePID;
}

PID& ControlBase::getDistanceVelPID()
{
	return *_distanceVelPID;
}

PID& ControlBase::getPosYPID()
{
	return *_posYPID;
}

PID& ControlBase::getPosXPID()
{
	return *_posXPID;
}

PID& ControlBase::getPosZPID()
{
	return *_posZPID;
}

PID& ControlBase::getVelYPID()
{
	return *_velYPID;
}

PID& ControlBase::getVelXPID()
{
	return *_velXPID;
}

PID& ControlBase::getVelZPID()
{
	return *_velZPID;
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
	double val = (_joyMsg.axes[_joyIndices->AXIS_LINEAR_Z] + 1) / 2.0;
	return val * _joyScales->LINEAR_Z;
}

double ControlBase::getThrustSpUnscaled()
{
	return (_joyMsg.axes[_joyIndices->AXIS_LINEAR_Z] + 1) / 2.0;
}

double ControlBase::getZPosSpManual()
{
	// TODO: Remove 0.1 magic number from here
	return _joyMsg.axes[_joyIndices->AXIS_LINEAR_Z] * 0.1;
}

double ControlBase::getYawScale()
{
	return _joyScales->ANGULAR_Z;
}

double ControlBase::getThrustScale()
{
	return _joyScales->LINEAR_Z;
}

const std::array<double, 3>& ControlBase::getCurrPosition()
{
	return _currentPosition;
}

const std::array<double, 3>& ControlBase::getCurrVelocity()
{
	return _currentVelocity;
}

void ControlBase::parametersCallback(
		plane_detection_ros::DistanceControlParametersConfig& configMsg,
		uint32_t level)
{
	ROS_WARN("ControlBase::parametersCallback()");
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

	_velYPID->set_kp(configMsg.k_p_vy);
	_velYPID->set_kd(configMsg.k_d_vy);
	_velYPID->set_ki(configMsg.k_i_vy);
	_velYPID->set_lim_high(configMsg.lim_high_vy);
	_velYPID->set_lim_low(configMsg.lim_low_vy);
	
	_posXPID->set_kp(configMsg.k_p_y);
	_posXPID->set_kd(configMsg.k_d_y);
	_posXPID->set_ki(configMsg.k_i_y);
	_posXPID->set_lim_high(configMsg.lim_high_y);
	_posXPID->set_lim_low(configMsg.lim_low_y);

	_velXPID->set_kp(configMsg.k_p_vy);
	_velXPID->set_kd(configMsg.k_d_vy);
	_velXPID->set_ki(configMsg.k_i_vy);
	_velXPID->set_lim_high(configMsg.lim_high_vy);
	_velXPID->set_lim_low(configMsg.lim_low_vy);

	_posZPID->set_kp(configMsg.k_p_z);
	_posZPID->set_kd(configMsg.k_d_z);
	_posZPID->set_ki(configMsg.k_i_z);
	_posZPID->set_lim_high(configMsg.lim_high_z);
	_posZPID->set_lim_low(configMsg.lim_low_z);

	_velZPID->set_kp(configMsg.k_p_vz);
	_velZPID->set_kd(configMsg.k_d_vz);
	_velZPID->set_ki(configMsg.k_i_vz);
	_velZPID->set_lim_high(configMsg.lim_high_vz);
	_velZPID->set_lim_low(configMsg.lim_low_vz);
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
	
	_distancePID->initializeParameters(nh, "/control/pitch");
	_distanceVelPID->initializeParameters(nh, "/control/pitch_rate");
	_posYPID->initializeParameters(nh, "/control/pos_y");
	_velYPID->initializeParameters(nh, "/control/vel_y");
	_posXPID->initializeParameters(nh, "/control/pos_y");
	_velXPID->initializeParameters(nh, "/control/vel_y");
	_posZPID->initializeParameters(nh, "/control/pos_z");
	_velZPID->initializeParameters(nh, "/control/vel_z");
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
	
	config.k_p_vy = _velYPID->get_kp();
	config.k_i_vy = _velYPID->get_ki();
	config.k_d_vy = _velYPID->get_kd();
	config.lim_low_vy = _velYPID->get_lim_low();
	config.lim_high_vy = _velYPID->get_lim_high();
	
	config.k_p_z = _posZPID->get_kp();
	config.k_i_z = _posZPID->get_ki();
	config.k_d_z = _posZPID->get_kd();
	config.lim_low_z = _posZPID->get_lim_low();
	config.lim_high_z = _posZPID->get_lim_high();

	config.k_p_vz = _velZPID->get_kp();
	config.k_i_vz = _velZPID->get_ki();
	config.k_d_vz = _velZPID->get_kd();
	config.lim_low_vz = _velZPID->get_lim_low();
	config.lim_high_vz = _velZPID->get_lim_high();

	ROS_INFO_STREAM(*_posZPID);
}
