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
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <uav_ros_control/PID.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>

// Own includes
#include <plane_detection_ros/DistanceControlParametersConfig.h>
#include <plane_detection_ros/control/JoyStructure.h>

/**
 * This class is used for defining Control subscribers and publishers.
 */
class ControlBase {
public:

	/**
	 * Default constructor. Used for reading ROS parameters and initalizing private variables.
	 */
	ControlBase();
	virtual ~ControlBase();

	/**	
	 * Distance callback function.
	 */
	void distanceCb(const std_msgs::Float64ConstPtr& message);

	/**
	 * Distance velocity callback function.
	 */
	void distanceVelCb(const std_msgs::Float64ConstPtr& message);

	/**
	 * Joystick callback function.
	 */
	void joyCb(const sensor_msgs::JoyConstPtr& message);

	/**
	 * IMU callback function for realistic control mode. 
	 * Calculates UAV yaw.
	 */
	void imuCbReal(const sensor_msgs::ImuConstPtr& message);

	/**
	 * Imu callback function for simulation control mode.
	 * Calculates UAV yaw.
	 */
	void imuCbSim(const nav_msgs::OdometryConstPtr& message);

	/**
	 * Plane normal callback function.
	 */
	void normalCb(const geometry_msgs::PoseStampedConstPtr& message);

	/**
	 * Callback for parameter server.
	 */
	void parametersCallback(
			plane_detection_ros::DistanceControlParametersConfig& configMsg,
			uint32_t level);

	/**
	 * Check if inspection is enabled.
	 */
	bool inspectionEnabledJoy();

	/**
	 * Return currently measured distance.
	 */
	double getDistanceMeasured();

	/**
	 * Get rate of change of measured distance.
	 */
	double getDistanceVelMeasured();

	/**
	 * Return reference to the PID object.
	 */
	PID& getPID();

	/**
	 * Return reference to velocity PID object.
	 */
	PID& getPID_vx();

	/**
	 * Return plane yaw angle, with respect to the UAV base frame.
	 */
	double getPlaneYaw();

	/**
	 * Return the current UAV yaw angle.
	 */
	double getUAVYaw();

	/**
	 * Return the value for current roll setpoint.
	 */
	double getRollSpManual();

	/**
	 * Return the value for current pitch setpoint.
	 */
	double getPitchSpManual();

	/**
	 * Return the value for current yaw setpoint.
	 */
	double getYawSpManual();
	/**
	 * Return the value for current thrust setpoint.
	 */
	double getThrustSpManual();

	/**
	 * Return the unscaled value for current thrust setpoint.
	 */
	double getThrustSpUnscaled();

	double getYawScale();
	
	/**
	 * Update configuration parameters on the given server.
	 */
	template <class T>
	void setReconfigureParameters(dynamic_reconfigure::Server<T>& server)
	{
		plane_detection_ros::DistanceControlParametersConfig configMsg;
		
		// Distance controller
		configMsg.k_p_x = _distancePID->get_kp();
		configMsg.k_i_x = _distancePID->get_ki();
		configMsg.k_d_x = _distancePID->get_kd();
		configMsg.lim_high_x = _distancePID->get_lim_high();
		configMsg.lim_low_x = _distancePID->get_lim_low();

		// Distance velocity controller
		configMsg.k_p_vx = _distanceRatePID->get_kp();
		configMsg.k_i_vx = _distanceRatePID->get_ki();
		configMsg.k_d_vx = _distanceRatePID->get_kd();
		configMsg.lim_high_vx = _distanceRatePID->get_lim_high();
		configMsg.lim_low_vx = _distanceRatePID->get_lim_low();

		server.updateConfig(configMsg);
	}

private:

	/** Distance PID controller */
	std::unique_ptr<PID> _distancePID;

	/** Distance velocity PID controller */
	std::unique_ptr<PID> _distanceRatePID;

	/** Current Joy message set in the /joy callback function. */
	sensor_msgs::Joy _joyMsg;

	/** Indices - Joy structure */
	std::unique_ptr<joy_control::JoyIndices> _joyIndices;

	/** Scale weights - Joy structure */
	std::unique_ptr<joy_control::ScaleWeights> _joyScales;

	/** Current distance measured value. Used both in sim and real mode. */
	double _distanceMeasured;

	double _distanceVelocityMeasured;

	/** Current UAV yaw angle. */
	double _uavYaw = 0;

	/** Yaw of the plane normal with respect to UAV base frame. */
	double _planeYaw = 0;

};

#endif /* CONTROL_BASE_H */
