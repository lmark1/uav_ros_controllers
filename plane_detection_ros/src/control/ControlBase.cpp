/*
 * ControlBase.cpp
 *
 *  Created on: Apr 11, 2019
 *      Author: lmark
 */
#include <uav_ros_control/control/ControlBase.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>

// Cpp includes
#include <iostream>
#include <vector>
#include <array>
#include <math.h>

uav_controller::ControlBase::ControlBase(ros::NodeHandle& nh)
{
	// Initialize all subscribers
	_subOdom = nh.subscribe("/odometry", 1,
		&uav_controller::ControlBase::odomCb, this);
	_subCarrotPose = nh.subscribe("/carrot/position", 1, 
		&uav_controller::ControlBase::carrotCb, this);
	_subTrajPoint = nh.subscribe("/uav/trajectory_point", 1,
		&uav_controller::ControlBase::trajPointCb, this);

	// Initialized all publishers
	_pubAttitudeSetpoint = 
		nh.advertise<mavros_msgs::AttitudeTarget>("/uav/attitude_setpoint", 1);
	_pubEulerSetpoint = 
		nh.advertise<geometry_msgs::Vector3>("/uav/euler_setpoint", 1);

	// Initialize references
	_currCarrotRef.transforms = std::vector<geometry_msgs::Transform>(1);
	_currCarrotRef.velocities = std::vector<geometry_msgs::Twist>(1);
	_currCarrotRef.accelerations = std::vector<geometry_msgs::Twist>(1);
}

uav_controller::ControlBase::~ControlBase()
{
}

void uav_controller::ControlBase::odomCb(const nav_msgs::OdometryConstPtr& message)
{
	_currentPosition[0] = message->pose.pose.position.x;
	_currentPosition[1] = message->pose.pose.position.y;
	_currentPosition[2] = message->pose.pose.position.z;

	_currentVelocity[0] = message->twist.twist.linear.x;
	_currentVelocity[1] = message->twist.twist.linear.y;
	_currentVelocity[2] = - message->twist.twist.linear.z;
}

void uav_controller::ControlBase::carrotCb(const geometry_msgs::PoseStampedConstPtr& msg)
{
	
}

void uav_controller::ControlBase::trajPointCb(
	const trajectory_msgs::MultiDOFJointTrajectoryPointConstPtr&)
{
	
}

const std::array<double, 3>& uav_controller::ControlBase::getCurrPosition()
{
	return _currentPosition;
}

const std::array<double, 3>& uav_controller::ControlBase::getCurrVelocity()
{
	return _currentVelocity;
}

void uav_controller::ControlBase::publishAttitudeTarget(int typeMask)
{

}

void uav_controller::ControlBase::setThrustSp(const double thrust)
{
	_attThrustSp[3] = thrust;
}

void uav_controller::ControlBase::setAttitudeSp(
	const double roll, const double pitch, const double yaw)
{
	_attThrustSp[0] = roll;
	_attThrustSp[1] = pitch;
	_attThrustSp[2] = yaw;
}

void uav_controller::ControlBase::publishEulerSp()
{
	geometry_msgs::Vector3 newMessage;
	newMessage.x = _attThrustSp[0];
	newMessage.y = _attThrustSp[1];
	newMessage.z = _attThrustSp[2];
	_pubEulerSetpoint.publish(newMessage);
}