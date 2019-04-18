/*
 * DistanceControl.h
 *
 *  Created on: Apr 11, 2019
 *      Author: lmark
 */

#ifndef CONTROL_BASE_H
#define CONTROL_BASE_H

// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <uav_ros_control/PID.h>
#include <geometry_msgs/Twist.h>

// Own includes
#include <plane_detection_ros/DistanceControlParametersConfig.h>
#include <plane_detection_ros/control/JoyStructure.h>

// Cpp includes
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
		_distanceMeasured (-1),
		_distancePID (new PID),
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

		// Make sure that thrust value is non negative
		if (_joyMsg.axes[_joyIndices->AXIS_LINEAR_Z] < 0)
			_joyMsg.axes[_joyIndices->AXIS_LINEAR_Z] = 0;
	}

	/**
	 * IMU callback function for realistic control mode. 
	 * Calculates UAV yaw.
	 */
	void imuCbReal(const sensor_msgs::ImuConstPtr& message)
	{
		_uavYaw = calculateYaw(
				message->orientation.x,
				message->orientation.y,
				message->orientation.z,
				message->orientation.w);
	}

		/**
	 * Imu callback function for simulation control mode.
	 * Calculates UAV yaw.
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
	 * Callback for parameter server.
	 */
	void parametersCallback(
			plane_detection_ros::DistanceControlParametersConfig& configMsg,
			uint32_t level)
	{
		_distancePID->set_kp(configMsg.k_p);
		_distancePID->set_kd(configMsg.k_d);
		_distancePID->set_ki(configMsg.k_i);
		_distancePID->set_lim_high(configMsg.lim_high);
		_distancePID->set_lim_low(configMsg.lim_low);
	}

	/**
	 * Update configuration parameters on the given server.
	 */
	template <class T>
	void setReconfigureParameters(dynamic_reconfigure::Server<T>& server)
	{
		plane_detection_ros::DistanceControlParametersConfig configMsg;
		configMsg.k_p = _distancePID->get_kp();
		configMsg.k_i = _distancePID->get_ki();
		configMsg.k_d = _distancePID->get_kd();
		configMsg.lim_high = _distancePID->get_lim_high();
		configMsg.lim_low = _distancePID->get_lim_low();
		server.updateConfig(configMsg);
	}

	/**
	 * Check if inspection is enabled.
	 */
	bool inspectionEnabledJoy()
	{
		return _joyMsg.buttons[_joyIndices->INSPECTION_MODE] == 1;
	}

	/**
	 * Return currently measured distance.
	 */
	double getDistanceMeasured()
	{
		return _distanceMeasured;
	}

	/**
	 * Return reference to the PID object.
	 */
	PID& getPID()
	{
		return *_distancePID;
	}

	/**
	 * Return plane yaw angle, with respect to the UAV base frame.
	 */
	double getPlaneYaw()
	{
		return _planeYaw;
	}

	/**
	 * Return the current UAV yaw angle.
	 */
	double getUAVYaw()
	{
		return _uavYaw;
	}

	/**
	 * Return the value for current roll setpoint.
	 */
	double getRollSpManual()
	{
		return _joyMsg.axes[_joyIndices->AXIS_LINEAR_Y] * _joyScales->LINEAR_Y;
	}

	/**
	 * Return the value for current pitch setpoint.
	 */
	double getPitchSpManual()
	{
		return _joyMsg.axes[_joyIndices->AXIS_LINEAR_X] * _joyScales->LINEAR_X;
	}

	/**
	 * Return the value for current yaw setpoint.
	 */
	double getYawSpManual()
	{
		return _joyMsg.axes[_joyIndices->AXIS_ANGULAR_YAW] * _joyScales->ANGULAR_Z;
	}

	/**
	 * Return the value for current thrust setpoint.
	 */
	double getThrustSpManual()
	{
		return _joyMsg.axes[_joyIndices->AXIS_LINEAR_Z] * _joyScales->LINEAR_Z;
	}

	/**
	 * Return the unscaled value for current thrust setpoint.
	 */
	double getThrustSpUnscaled()
	{
		return _joyMsg.axes[_joyIndices->AXIS_LINEAR_Z];
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

	/** Distance PID controller */
	std::unique_ptr<PID> _distancePID;

	/** Current Joy message set in the /joy callback function. */
	sensor_msgs::Joy _joyMsg;

	/** Indices - Joy structure */
	std::unique_ptr<joy_control::JoyIndices> _joyIndices;

	/** Scale weights - Joy structure */
	std::unique_ptr<joy_control::ScaleWeights> _joyScales;

	/** Current distance measured value. Used both in sim and real mode. */
	double _distanceMeasured;

	/** Current UAV yaw angle. */
	double _uavYaw = 0;

	/** Yaw of the plane normal with respect to UAV base frame. */
	double _planeYaw = 0;
};

#endif /* CONTROL_BASE_H */