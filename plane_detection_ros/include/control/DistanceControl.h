/*
 * DistanceControl.h
 *
 *  Created on: Apr 13, 2019
 *      Author: lmark
 */

#ifndef DISTANCE_CONTROL_H
#define DISTANCE_CONTROL_H

#include "control/ControlBase.h"

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

class DistanceControl : public ControlBase {

public:

	/**
	 * Default contructor which initializes the control mode.
	 *
	 * @param mode - Given control mode
	 */
	DistanceControl(DistanceControlMode mode);
	virtual ~DistanceControl();

private:
	DistanceControlMode _mode;
};

#endif /* DISTANCE_CONTROL_H */
