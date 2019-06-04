/*
 * DistanceControl.cpp
 *
 *  Created on: Apr 11, 2019
 *      Author: lmark
 */

#include <plane_detection_ros/control/ControlBase.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>

// Cpp includes
#include <iostream>
#include <vector>
#include <array>
#include <math.h>

control_base::ControlBase::ControlBase(ros::NodeHandle& nh)
{
	// Initialize all subscribers
	_subImuReal = nh.subscribe("/real/imu", 1,
		&control_base::ControlBase::imuCbReal, this);
	_subOdomReal = nh.subscribe("/real/odom", 1,
		&control_base::ControlBase::odomCbReal, this);
	_subOdomSim = nh.subscribe("/sim/odom", 1, 
		&control_base::ControlBase::odomCbSim, this);

	// Initialized all publishers
	_pubSpReal = nh.advertise<mavros_msgs::AttitudeTarget>("/real/attitude_sp", 1);
	_pubSpSim = nh.advertise<mav_msgs::RollPitchYawrateThrust>("/sim/attitude_sp", 1);
	_pubEulerSp = nh.advertise<geometry_msgs::Vector3>("/euler_sp", 1);

	// Initialize global parameter
	_global = true;
	bool initialized = nh.getParam("/control/global", _global);
	ROS_INFO_STREAM("ControlBase::ControlBase() - global mode: " 
		<< std::boolalpha << _global << std::endl);
	if (!initialized)
	{
		ROS_FATAL("ControlBase::ControlBase() - global mode flag not initialized.");
		throw std::runtime_error("ControlBase parameters not set");
	}
}

control_base::ControlBase::~ControlBase()
{
}

void control_base::ControlBase::imuCbReal(const sensor_msgs::ImuConstPtr& message)
{
	_uavYaw = calculateYaw(
		message->orientation.x,
		message->orientation.y,
		message->orientation.z,
		message->orientation.w);
}

void control_base::ControlBase::odomCbSim(const nav_msgs::OdometryConstPtr& message)
{
	_uavYaw = calculateYaw(
		message->pose.pose.orientation.x,
		message->pose.pose.orientation.y,
		message->pose.pose.orientation.z,
		message->pose.pose.orientation.w);

	// Rotate simulated position from global to local coordinate system
	rotateVector(
		message->pose.pose.position.x,
		message->pose.pose.position.y,
		message->pose.pose.position.z,
		_currentPosition);

	// Odometry simulation callback, velocity vector is already in the Local base frame
	_currentVelocity[0] = message->twist.twist.linear.x;
	_currentVelocity[1] = message->twist.twist.linear.y;
	_currentVelocity[2] = message->twist.twist.linear.z;
}

void control_base::ControlBase::odomCbReal(const nav_msgs::OdometryConstPtr& message)
{
	_currentPosition[0] = message->pose.pose.position.x;
	_currentPosition[1] = message->pose.pose.position.y;
	_currentPosition[2] = message->pose.pose.position.z;

	_currentVelocity[0] = message->twist.twist.linear.x;
	_currentVelocity[1] = message->twist.twist.linear.y;
	_currentVelocity[2] = - message->twist.twist.linear.z;
}

bool control_base::ControlBase::getGlobalFlag()
{
	return _global;
}

void control_base::ControlBase::rotateVector(
	const double x, const double y, const double z, std::array<double, 3>& vector)
{
	vector[0] = x * cos(-_uavYaw) - y * sin(-_uavYaw);
	vector[1] = x * sin(-_uavYaw) + y * cos(-_uavYaw);
	vector[2] = z;
}

double control_base::ControlBase::getUAVYaw()
{
	return _uavYaw;
}

const std::array<double, 3>& control_base::ControlBase::getCurrPosition()
{
	return _currentPosition;
}

const std::array<double, 3>& control_base::ControlBase::getCurrVelocity()
{
	return _currentVelocity;
}

double control_base::ControlBase::calculateYaw(double qx, double qy, double qz, double qw)
{
	return atan2(
			2 * (qw * qz + qx * qy),
			qw * qw + qx * qx - qy * qy - qz * qz);
}

void control_base::ControlBase::publishAttitudeSim(
	const std::array<double, 4>& attThrustSp, double yawRateSp)
{
	mav_msgs::RollPitchYawrateThrust newMessage;
	newMessage.header.stamp = ros::Time::now();
	newMessage.roll = attThrustSp[0];
	newMessage.pitch = attThrustSp[1];
	newMessage.yaw_rate = yawRateSp;
	newMessage.thrust = geometry_msgs::Vector3();
	newMessage.thrust.x = 0;
	newMessage.thrust.y = 0;
	newMessage.thrust.z = attThrustSp[3];
	_pubSpSim.publish(newMessage);
}

void control_base::ControlBase::publishAttitudeSim(double thrustScale)
{	
	// Delegate to the general method
	publishAttitudeSim( 
		std::array<double, 4> 
		{
			_attThrustSp[0], _attThrustSp[1], 0, _attThrustSp[3] * thrustScale	
		},
		_attThrustSp[2]);
}

void control_base::ControlBase::publishAttitudeReal(
	const std::array<double, 4>& attThrustSp, double yawRateSp, int typeMask)
{
	tf2::Quaternion q;
	q.setEulerZYX(attThrustSp[2], attThrustSp[1], attThrustSp[0]);

	mavros_msgs::AttitudeTarget newMessage;
	newMessage.header.stamp = ros::Time::now();
	newMessage.type_mask = typeMask;
	newMessage.body_rate.z = yawRateSp;
	newMessage.orientation.x = q.getX();
	newMessage.orientation.y = q.getY();
	newMessage.orientation.z = q.getZ();
	newMessage.orientation.w = q.getW();
	newMessage.thrust = attThrustSp[3];
	_pubSpReal.publish(newMessage);
}

void control_base::ControlBase::publishAttitudeReal()
{
	// Delegate to the general method
	publishAttitudeReal(
		std::array<double, 4> 
		{
			_attThrustSp[0], _attThrustSp[1], 0, _attThrustSp[3]
		}, 
		_attThrustSp[2], 
		MASK_IGNORE_RP_RATE);
}

void control_base::ControlBase::setThrustSp(const double thrust)
{
	_attThrustSp[3] = thrust;
}

void control_base::ControlBase::setAttitudeSp(
	const double roll, const double pitch, const double yaw)
{
	_attThrustSp[0] = roll;
	_attThrustSp[1] = pitch;
	_attThrustSp[2] = yaw;
}

void control_base::ControlBase::overridePitch(const double pitch)
{
	_attThrustSp[1] = pitch;
}

void control_base::ControlBase::overrideRoll(const double roll)
{
	_attThrustSp[0] = roll;
}

void control_base::ControlBase::overrideYaw(const double yaw)
{
	_attThrustSp[2] = yaw;
}

const std::array<double, 4>& control_base::ControlBase::getAttThrustSp()
{
	return _attThrustSp;
}

void control_base::ControlBase::publishEulerSp()
{
	geometry_msgs::Vector3 newMessage;
	newMessage.x = _attThrustSp[0];
	newMessage.y = _attThrustSp[1];
	newMessage.z = _attThrustSp[2];
	_pubEulerSp.publish(newMessage);
}