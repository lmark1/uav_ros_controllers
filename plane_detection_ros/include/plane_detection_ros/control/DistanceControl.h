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
	DistanceControl(DistanceControlMode mode);
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
	void calculateAttitudeTarget(double dt);
	
	/**
	 * Calculate appropriate attitude setpoint from the "carrot" control input.
	 */
	void calculateCarrotSetpoint(double dt);

	/**
	 * Publish current control state.
	 *
	 * @param pub - Given Int32 publisher
	 */
	void publishState(ros::Publisher& pub);

	/**
	 * Publish attitude setpoint on the given topic.
	 * If in simulation mode, publisher is expected to be Vector3.
	 * If in real mode, publisher is expected to be mavros_msgs::AttitudeTarget.
	 */
	void publishAttSp(ros::Publisher& pub);

	/**
	 * Publish distance setpoint as a std_msgs::Float64 message.
	 */
	void publishDistSp(ros::Publisher& pub);
	
	/**
	 * Publish distance velocity setpoint as a Float64 ROS message.
	 */
	void publishDistVelSp(ros::Publisher& pub);

	/**
	 * Publish setpoint euler angles as a Vector3 ROS message.
	 */
	void publishEulerSp(ros::Publisher& pub);

	/**
	 * Publish carrot position setpoint as a Vector3 ROS message.
	 */
	void publishPosSp(ros::Publisher& pub);

	/**
	 * Publish carrot velocity setpoint as a Vector3 ROS message.
	 */ 
	void publishVelSp(ros::Publisher& pub);

	/**
	 * Publish local position mesured value as a Vector3 ROS message.
	 */
	void publishPosMv(ros::Publisher& pub);

	/**
	 * Publish measured local velocity value as a Vector3 ROS message.
	void publishVelMv(ros::Publisher& pub);

	/**
	 * Return true if in inspection state, otherwise false.
	 */
	bool inInspectionState();

	/**
	 * Do all the parameter initialization here.
	 */
	virtual void initializeParameters(ros::NodeHandle& nh) override;

	/**
	 * Callback function used for setting various parameters.
	 */
	virtual void parametersCallback(
			plane_detection_ros::DistanceControlParametersConfig& configMsg,
			uint32_t level) override;

	/**
	 * Set reconfigure parameters in the given config object.
	 */
	virtual void setReconfigureParameters(plane_detection_ros::DistanceControlParametersConfig& config) override;

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

	/** Current control mode. */
	DistanceControlMode _mode;

	/** Current control state */
	DistanceControlState _currState;

	/** Flag used to detect when deactivate inspection is requested. */
	bool _deactivateInspection;

	/** True if inspection state was requested and denied, false otherwise. */
	bool _inspectionRequestFailed;

	/** Distance setpoint value */
	double _distSp;

	/** Distance velocity setpoint value. */
	double _distVelSp;

	/** Attitude setpoint array. */
	std::array<double, 43> _attThrustSp {0.0, 0.0, 0.0, 0.0};

	/** Carrot setpoint position array. */
	std::array<double, 3> _carrotPos {0.0, 0.0, 0.0};

	/** Carrot setpoint velocity array. */
	std::array<double, 3> _carrotVel {0.0, 0.0, 0.0};

	/** Value from 0 to 1, hover thrust */
	double _hoverThrust;
};

#endif /* DISTANCE_CONTROL_H */
