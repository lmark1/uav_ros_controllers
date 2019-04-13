/*
 * DistanceControl.h
 *
 *  Created on: Apr 11, 2019
 *      Author: lmark
 */

#ifndef CONTROL_BASE_H
#define CONTROL_BASE_H

#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

/**
 * This class is used for defining Control subscribers and publishers.
 */
class ControlBase {
public:

	/**
	 * Default constructor.
	 */
	ControlBase()
	{
	}

	virtual ~ControlBase()
	{
	}

	/**
	 * Distance callback function.
	 */
	void distanceCb(const std_msgs::Float64ConstPtr& message)
	{
		_distMsg = *message;
	}

	/**
	 * Joystick callback function.
	 */
	void joyCb(const sensor_msgs::JoyConstPtr& message)
	{
		_joyMsg = *message;
	}

	/**
	 * Plane normal callback function.
	 */
	void normalCb(const geometry_msgs::PoseStampedConstPtr& message)
	{
		_planeNormal = *message;
	}

	/**
	 * IMU callback function for realistic control mode.
	 */
	void imuCbReal(const sensor_msgs::ImuConstPtr& message)
	{
		_imuMsgReal = *message;
	}

	/**
	 * Imu callback function for simulation control mode.
	 */
	void imuCbSim(const nav_msgs::OdometryConstPtr& message)
	{
		_odomMsgSim = *message;
	}

private:

	/**
	 * Current Joy message received. Used both in sim and real mode.
	 */
	sensor_msgs::Joy _joyMsg;

	/**
	 * Current distance measured value. Used both in sim and real mode.
	 */
	std_msgs::Float64 _distMsg;

	/**
	 * Current IMU measured value. Used only is real mode.
	 */
	sensor_msgs::Imu _imuMsgReal;

	/**
	 * Current Odometry value. Used only in simulation mode.
	 */
	nav_msgs::Odometry _odomMsgSim;

	/**
	 * Current plane normal.
	 */
	geometry_msgs::PoseStamped _planeNormal;

};

#endif /* CONTROL_BASE_H */
