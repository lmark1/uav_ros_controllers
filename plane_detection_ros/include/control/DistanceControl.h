/*
 * DistanceControl.h
 *
 *  Created on: Apr 13, 2019
 *      Author: lmark
 */

#ifndef DISTANCE_CONTROL_H
#define DISTANCE_CONTROL_H

#include "control/ControlBase.h"

// ROS Includes
#include <ros/ros.h>

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
	REALISTIC
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
	 * Default contructor which initializes the control mode.
	 *
	 * @param mode - Given control mode
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
	 * Publish current control state.
	 *
	 * @param pub - Given Int32 publisher
	 */
	void publishState(ros::Publisher& pub);

private:

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

};

#endif /* DISTANCE_CONTROL_H */
