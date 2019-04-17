/*
 * DistanceControl.h
 *
 *  Created on: Apr 13, 2019
 *      Author: lmark
 */

#ifndef DISTANCE_CONTROL_H
#define DISTANCE_CONTROL_H

#include "plane_detection_ros/control/ControlBase.h"

// ROS Includes
#include <ros/ros.h>

#include <iostream>
#include <array>

/**
 * Define control modes used in DistanceControl algorithm.
 */
enum DistanceControlMode
{
	/**
	 * Simulation control mode.
	 */
	SIMULATION,

	/**
	 * Realistic control mode.
	 */
	REAL
};

enum DistanceControlState
{
	/**
	 * User controls the UAV manually using joystick commands.
	 */
	MANUAL,

	/**
	 * User controls the UAV while it performs inspection, maintaining
	 * the distance from the callback function feedback loop.
	 */
	INSPECTION
};

class DistanceControl : public ControlBase {

public:

	/**
	 * Defualt DistanceControl constructor. 
	 * 
	 * @param mode 	Defines control mode
	 * @param kp	Distance controller proportional gain
	 * @param ki	Distance controller integrator gain
	 * @param kd	Distance controller derivator gain
	 * @param lim_low	Lower saturation limit for for the PID integrator
	 * @param lim_high 	Higher saturation limit for the PID integrator
	 */	
	DistanceControl(DistanceControlMode mode, double kp, double ki, double kd, 
		double limLow, double limHigh);
	virtual ~DistanceControl();

	/**
	 * Change the state back to manual if received distance is invalid.
	 * Change the state to inspection mode if appropriate joystick command is
	 * given.
	 */
	void detectStateChange();

	/**
	 * Calculate appropriate attitude setpoint.
	 *
	 * @param dt - Given discretization time.
	 */
	void calculateSetpoint(double dt);

	/**
	 * Publish current control state.
	 *
	 * @param pub - Given Int32 publisher
	 */
	void publishState(ros::Publisher& pub);

	/**
	 * If in simulation mode, publisher is expected to be Vector3.
	 * If in real mode, publisher is expected to be mavros_msgs::AttitudeTarget.
	 */
	void publishSetpoint(ros::Publisher& pub);

	/**
	 * Publish distance setpoint as a std_msgs::Float64 message.
	 */
	void publishDistanceSetpoint(ros::Publisher& pub);

	/**
	 * Return true if in inspection state, otherwise false.
	 */
	bool inInspectionState();

private:

	/**
	 * Perform all necessary steps in order to deactivate inspection mode.
	 */
	void deactivateInspection();

	/**
	 * Returns true if inspection is requested, otherwise return false.
	 */
	bool inspectionRequested();

	/**
	 * Returns true if inspection failed, otherwise return false.
	 */
	bool inspectionFailed();

	/**
	 * Returns true if manual mode is requested, otherwise return false.
	 */
	bool manualRequested();

	/**
	 * Current control mode.
	 */
	DistanceControlMode _mode;

	/**
	 * Current control state
	 */
	DistanceControlState _currState;

	/**
	 * Flag used to detect when deactivate inspection is requested.
	 */
	bool _deactivateInspection;

	/**
	 * Referent distance value.
	 */
	double _distRef;

	/**
	 * Attitude setpoint array.
	 */
	std::array<double, 3> _attitudeSetpoint {0.0, 0.0, 0.0};
};

#endif /* DISTANCE_CONTROL_H */
