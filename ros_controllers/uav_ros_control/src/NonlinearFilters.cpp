/*
 * NonlinearFilters.cpp
 *
 *  Created on: Oct 11, 2018
 *      Author: lmark
 */

#include <uav_ros_control/NonlinearFilters.h>

double nonlinear_filters::saturation(
		double value,
		double lowLimit,
		double highLimit)
{
	if (value > highLimit) { return highLimit; }
	else if (value < lowLimit) { return lowLimit; }
	else { return value; }
}

double nonlinear_filters::deadzone(
		double value,
		double lowLimit,
		double highLimit)
{
	if (value < highLimit && value > lowLimit) { return 0; }
	else { return value; }
}

double nonlinear_filters::filterPT1(
		double previousValue,
		double currentValue,
		double T,
		double Ts,
		double K)
{
	double a = T / (T + Ts);
	double b = K * Ts / (T + Ts);

	return (a * previousValue + b * currentValue);
}


