/*
 * DistanceControl.h
 *
 *  Created on: Apr 11, 2019
 *      Author: lmark
 */

#ifndef DISTANCE_CONTROL_H
#define DISTANCE_CONTROL_H

#include <std_msgs/Float64.h>

/**
 * This class is used for UAV distance control.
 */
class DistanceControl {
public:
	DistanceControl();
	virtual ~DistanceControl();

	/**
	 * Distance callback function.
	 */
	void distanceCallback(const std_msgs::Float64& message);


};

#endif /* DISTANCE_CONTROL_H */
