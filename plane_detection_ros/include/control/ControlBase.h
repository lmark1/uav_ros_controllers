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
#include <uav_ros_control/PID.h>
#include <plane_detection_ros/DistanceControlParametersConfig.h>

/**
 * This class is used for defining Control subscribers and publishers.
 */
class ControlBase {
public:

	/**
	 * Default constructor.
	 */
	ControlBase():
		_distance(-1)
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
		_distance = message->data;
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

	/**
	 * Callback for parameter server.
	 */
	void parametersCallback(
			plane_detection_ros::DistanceControlParametersConfig& configMsg,
			uint32_t level)
	{
		_distancePID.set_kp(configMsg.k_p);
		_distancePID.set_kd(configMsg.k_d);
		_distancePID.set_ki(configMsg.k_i);
		_distancePID.set_lim_high(configMsg.lim_high);
		_distancePID.set_lim_low(configMsg.lim_low);
	}

	sensor_msgs::Joy getJoyMsg()
	{
		return _joyMsg;
	}

	double getDistance()
	{
		return _distance;
	}

	sensor_msgs::Imu getIMUMsg()
	{
		return _imuMsgReal;
	}

	nav_msgs::Odometry getOdomMsg()
	{
		return _odomMsgSim;
	}

	geometry_msgs::PoseStamped getPlaneNormal()
	{
		return _planeNormal;
	}

	PID getPID()
	{
		return _distancePID;
	}


private:

	/**
	 * Distance PID controller
	 */
	PID _distancePID;

	/**
	 * Current Joy message received. Used both in sim and real mode.
	 */
	sensor_msgs::Joy _joyMsg;

	/**
	 * Current distance measured value. Used both in sim and real mode.
	 */
	double _distance;

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
