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
	_subOdom = nh.subscribe("odometry", 1,
		&uav_controller::ControlBase::odomCb, this);
	_subReference = nh.subscribe("uav/trajectory_pojnt", 1, 
		&uav_controller::ControlBase::trajPointCb, this);

	// Initialized all publishers
	_pubAttitudeSetpoint = 
		nh.advertise<mavros_msgs::AttitudeTarget>("uav/attitude_target", 1);
	_pubEulerSetpoint = 
		nh.advertise<geometry_msgs::Vector3>("uav/euler_setpoint", 1);

	// Initialize references
	_currentReference.transforms = std::vector<geometry_msgs::Transform>(1);
	_currentReference.velocities = std::vector<geometry_msgs::Twist>(1);
	_currentReference.accelerations = std::vector<geometry_msgs::Twist>(1);
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

void uav_controller::ControlBase::trajPointCb(
    const trajectory_msgs::MultiDOFJointTrajectoryPointConstPtr& msg)
{
    if (msg->transforms.size() == 0 || msg->velocities.size() == 0 || msg->accelerations.size() == 0)
    {
        ROS_WARN("ControlBase::trajPointCb - Trajectory point incomplete.");
        return;
    } 

    // Position
    _currentReference.transforms[0].translation.x = msg->transforms[0].translation.x;
    _currentReference.transforms[0].translation.y = msg->transforms[0].translation.y;
    _currentReference.transforms[0].translation.z = msg->transforms[0].translation.z;

    // Orientation
    _currentReference.transforms[0].rotation.w = msg->transforms[0].rotation.w;
    _currentReference.transforms[0].rotation.x = msg->transforms[0].rotation.x;
    _currentReference.transforms[0].rotation.y = msg->transforms[0].rotation.y;
    _currentReference.transforms[0].rotation.z = msg->transforms[0].rotation.z;

    // Linear velocity
    _currentReference.velocities[0].linear.x = msg->velocities[0].linear.x;
    _currentReference.velocities[0].linear.y = msg->velocities[0].linear.y;
    _currentReference.velocities[0].linear.z = msg->velocities[0].linear.z;

    // Angular velocity
    _currentReference.velocities[0].angular.x = msg->velocities[0].angular.x;
    _currentReference.velocities[0].angular.y = msg->velocities[0].angular.y;
    _currentReference.velocities[0].angular.z = msg->velocities[0].angular.z;

    // Linear acceleration
    _currentReference.accelerations[0].linear.x = msg->accelerations[0].linear.x;
    _currentReference.accelerations[0].linear.y = msg->accelerations[0].linear.y;
    _currentReference.accelerations[0].linear.z = msg->accelerations[0].linear.z;

    // Angular acceleration
    _currentReference.accelerations[0].angular.x = msg->accelerations[0].angular.x;
    _currentReference.accelerations[0].angular.y = msg->accelerations[0].angular.y;
    _currentReference.accelerations[0].angular.z = msg->accelerations[0].angular.z;
}

const std::array<double, 3>& uav_controller::ControlBase::getCurrPosition()
{
	return _currentPosition;
}

const std::array<double, 3>& uav_controller::ControlBase::getCurrVelocity()
{
	return _currentVelocity;
}

void uav_controller::ControlBase::publishAttitudeTarget(int typeMask, double yawRate = 0)
{
    tf2::Quaternion q;
	q.setEulerZYX(_attThrustSp[2], _attThrustSp[1], _attThrustSp[0]);

    mavros_msgs::AttitudeTarget newMessage;
	newMessage.header.stamp = ros::Time::now();
	newMessage.type_mask = typeMask;
	newMessage.body_rate.z = yawRate;
	newMessage.orientation.x = q.getX();
	newMessage.orientation.y = q.getY();
	newMessage.orientation.z = q.getZ();
	newMessage.orientation.w = q.getW();
	newMessage.thrust = _attThrustSp[3];
	_pubAttitudeSetpoint.publish(newMessage);
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