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
#include <geometry_msgs/Twist.h>

#include <plane_detection_ros/DistanceControlParametersConfig.h>

#include <iostream>
#include <vector>
#include <array>
#include <math.h>

/**
 * This class is used for defining Control subscribers and publishers.
 */
class ControlBase {
public:

	/**
	 * Default constructor.
	 */
	ControlBase():
		_distanceMeasured(-1),
		MASTER_JOY_INDEX(5),
		INSPECTION_JOY_INDEX(4)
	{
		// Initialize some default values
		_joyMsg.buttons = std::vector<int> (10, 0);
		_joyMsg.axes = std::vector<float> (10, 0.0);
	}

	virtual ~ControlBase()
	{
	}

	/**
	 * Distance callback function.
	 */
	void distanceCb(const std_msgs::Float64ConstPtr& message)
	{
		_distanceMeasured = message->data;
	}

	/**
	 * Joystick callback function.
	 */
	void joyCb(const sensor_msgs::JoyConstPtr& message)
	{
		_joyMsg = *message;
	}

	/**
	 * IMU callback function for realistic control mode.
	 */
	void imuCbReal(const sensor_msgs::ImuConstPtr& message)
	{
		// TODO: Do something here
		_imuMsgReal = *message;
	}

	/**
	 * Plane normal callback function.
	 */
	void normalCb(const geometry_msgs::PoseStampedConstPtr& message)
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

	/**
	 * Imu callback function for simulation control mode.
	 */
	void imuCbSim(const nav_msgs::OdometryConstPtr& message)
	{
		_uavYaw = calculateYaw(
				message->pose.pose.orientation.x,
				message->pose.pose.orientation.y,
				message->pose.pose.orientation.z,
				message->pose.pose.orientation.w);
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

	/**
	 * Command velocity callback.
	 */
	void cmdVelCb(const geometry_msgs::TwistConstPtr& twistMsg)
	{
		_rollSetpoint = twistMsg->linear.y;
	}

	/**
	 * Check if inspection is enabled.
	 */
	bool inspectionEnabledJoy()
	{
		return _joyMsg.buttons[INSPECTION_JOY_INDEX] == 1;
	}

	/**
	 * Return currently measured distance.
	 */
	double getDistanceMeasured()
	{
		ROS_DEBUG("getDistanceMeasured()");
		return _distanceMeasured;
	}

	/**
	 * Calculate yaw angle setpoint;
	 */
	double getYawSetpoint()
	{
		return _uavYaw + _planeYaw;
	}

	/**
	 * Return current roll setpoint.
	 */
	double getRollSetpoint()
	{
		return _rollSetpoint;
	}

	sensor_msgs::Imu getIMUMsg()
	{
		return _imuMsgReal;
	}

	PID getPID()
	{
		return _distancePID;
	}


private:

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
	double _distanceMeasured;

	/**
	 * Roll setpoint - when in inspection mode.
	 */
	double _rollSetpoint = 0;

	/**
	 * Current IMU measured value. Used only is real mode.
	 */
	sensor_msgs::Imu _imuMsgReal;

	/**
	 * Current UAV yaw angle.
	 */
	double _uavYaw = 0;

	/**
	 * Yaw of the plane normal with respect to UAV base frame.
	 */
	double _planeYaw = 0;

	/**
	 * Index of buttons[] array for changing to inspection mode.
	 */
	int INSPECTION_JOY_INDEX;

	/**
	 * Index of buttons[] array for setting master control mode.
	 */
	int MASTER_JOY_INDEX;
};

#endif /* CONTROL_BASE_H */
