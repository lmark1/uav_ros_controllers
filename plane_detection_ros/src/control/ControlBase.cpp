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

/**
 *  wrap x -> [0,max) 
 */
double wrapMax(double x, double max);

control_base::ControlBase::ControlBase()
{
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

	// Odometry simulation callback, velocity vector is alread in the Local base frame
	_currentVelocity[0] = message->twist.twist.linear.x;
	_currentVelocity[1] = message->twist.twist.linear.y;
	_currentVelocity[2] = message->twist.twist.linear.z;
}

void control_base::ControlBase::odomCbReal(const nav_msgs::OdometryConstPtr& message)
{
	// Rotate real position from global to local coordinate system
	rotateVector(
		message->pose.pose.position.x,
		message->pose.pose.position.y,
		message->pose.pose.position.z,
		_currentPosition);

	/**
	 * For some reason velocity z-component from Kopterworx UAV odometry
	 * sensor is signed incorrectly.
	 */
	rotateVector(
		message->twist.twist.linear.x,
		message->twist.twist.linear.y,
		- message->twist.twist.linear.z, 
		_currentVelocity);
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
	ros::Publisher& pub, const std::array<double, 4>& attThrustSp, double yawRateSp)
{
	mav_msgs::RollPitchYawrateThrust newMessage;
	newMessage.roll = attThrustSp[0];
	newMessage.pitch = attThrustSp[1];
	newMessage.yaw_rate = yawRateSp;
	newMessage.thrust = geometry_msgs::Vector3();
	newMessage.thrust.x = 0;
	newMessage.thrust.y = 0;
	newMessage.thrust.z = attThrustSp[3];
	pub.publish(newMessage);
}

void control_base::ControlBase::publishAttitudeSim(ros::Publisher& pub, double thrustScale)
{
	mav_msgs::RollPitchYawrateThrust newMessage;
	newMessage.roll = _attThrustSp[0];
	newMessage.pitch = _attThrustSp[1];
	newMessage.yaw_rate = _attThrustSp[2];
	newMessage.thrust = geometry_msgs::Vector3();
	newMessage.thrust.x = 0;
	newMessage.thrust.y = 0;
	newMessage.thrust.z = _attThrustSp[3] * thrustScale;
	pub.publish(newMessage);
}

void control_base::ControlBase::publishAttitudeReal(
	ros::Publisher& pub, const std::array<double, 4>& attThrustSp, double yawRateSp, int typeMask)
{
	tf2::Quaternion q;
	q.setEulerZYX(attThrustSp[2], attThrustSp[1], attThrustSp[0]);

	mavros_msgs::AttitudeTarget newMessage;
	newMessage.type_mask = typeMask;
	newMessage.body_rate.z = yawRateSp;
	newMessage.orientation.x = q.getX();
	newMessage.orientation.y = q.getY();
	newMessage.orientation.z = q.getZ();
	newMessage.orientation.w = q.getW();
	newMessage.thrust = attThrustSp[3];
	pub.publish(newMessage);
}

void control_base::ControlBase::publishAttitudeReal(ros::Publisher& pub)
{
	tf2::Quaternion q;
	q.setEulerZYX(0, _attThrustSp[1], _attThrustSp[0]);

	mavros_msgs::AttitudeTarget newMessage;
	newMessage.type_mask = MASK_IGNORE_RP_RATE;
	newMessage.body_rate.z = _attThrustSp[2];
	newMessage.orientation.x = q.getX();
	newMessage.orientation.y = q.getY();
	newMessage.orientation.z = q.getZ();
	newMessage.orientation.w = q.getW();
	newMessage.thrust = _attThrustSp[3];
	pub.publish(newMessage);
}

double control_base::ControlBase::wrapMinMax(double x, double min, double max)
{
    return min + wrapMax(x - min, max - min);
}

void control_base::ControlBase::setThrustSp(const double thrust)
{
	_attThrustSp[3] = thrust;
}

void control_base::ControlBase::setAttitudeSp(const double roll, const double pitch, const double yaw)
{
	_attThrustSp[0] = roll;
	_attThrustSp[1] = pitch;
	_attThrustSp[2] = yaw;
}

const std::array<double, 4>& control_base::ControlBase::getAttThrustSp()
{
	return _attThrustSp;
}

void control_base::ControlBase::publishEulerSp(ros::Publisher& pub)
{
	geometry_msgs::Vector3 newMessage;
	newMessage.x = _attThrustSp[0];
	newMessage.y = _attThrustSp[1];
	newMessage.z = _attThrustSp[2];
	pub.publish(newMessage);
}

double wrapMax(double x, double max)
{
    /* integer math: `(max + x % max) % max` */
    return fmod(max + fmod(x, max), max);
}

control_base::ControlBase* control_base::ControlBase::getBasePointer()
{
	return this;
}