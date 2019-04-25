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

//Cpp includes
#include <array>

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
	 * Position callback for real control mode.
	 */
	void posCbReal(const geometry_msgs::PoseStampedConstPtr& message);

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
	PID& getPitchPID();

	/**
	 * Return reference to velocity PID object.
	 */
	PID& getPitchRatePID();

	/**
	 * Return a reference to the y position PID object.
	 */
	PID& getPosYPID();

	/**
	 * Return a reference to the z position PID object.
	 */
	PID& getPosZPID();

	/**
	 * Return a reference to the y velocity PID object.
	 */
	PID& getVelYPID();

	/**
	 * Return a reference to the z velocity PID object.
	 */
	PID& getVelZPID();

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

	/**
	 * Return scale value for yaw control input.
	 */
	double getYawScale();

	/**
	 * Return constant reference to the current position.
	 */
	const std::array<double, 3>& getCurrPosition();

	/**
	 * Do all the parameter initialization here.
	 */
	virtual void initializeParameters(ros::NodeHandle& nh);

	/**
	 * Callback function used for setting various parameters.
	 */
	virtual void parametersCallback(
			plane_detection_ros::DistanceControlParametersConfig& configMsg,
			uint32_t level);

	/**
	 * Set reconfigure parameters in the given config object.
	 */
	virtual void setReconfigureParameters(plane_detection_ros::DistanceControlParametersConfig& config);

private:

	/**
	 * Update local position.
	 */
	void updatePosition(double x, double y, double z);

	/** Distance PID controller */
	std::unique_ptr<PID> _distancePID;

	/** Distance velocity PID controller */
	std::unique_ptr<PID> _distanceVelPID;

	/** PID controller for position along the y-axis.*/
	std::unique_ptr<PID> _posYPID;
	
	/** PID controller for velocity along the y-axis */
	std::unique_ptr<PID> _velYPID;

	/** PID controller for position along the z-axis.*/
	std::unique_ptr<PID> _posZPID;

	/** PID controller for velocity along the y-axis */
	std::unique_ptr<PID> _velZPID;

	/** Current Joy message set in the /joy callback function. */
	sensor_msgs::Joy _joyMsg;

	/** Indices - Joy structure */
	std::unique_ptr<joy_control::JoyIndices> _joyIndices;

	/** Scale weights - Joy structure */
	std::unique_ptr<joy_control::ScaleWeights> _joyScales;

	/** Current LOCAL position vector. */
	std::array<double, 3> _currentPosition {0.0, 0.0, 0.0};

	/** Current distance measured value. Used both in sim and real mode. */
	double _distanceMeasured;

	/** Currently measured distance velocity. Used both in sim and real mode. */
	double _distanceVelocityMeasured;

	/** Current UAV yaw angle. */
	double _uavYaw = 0;

	/** Yaw of the plane normal with respect to UAV base frame. */
	double _planeYaw = 0;

};

#endif /* CONTROL_BASE_H */