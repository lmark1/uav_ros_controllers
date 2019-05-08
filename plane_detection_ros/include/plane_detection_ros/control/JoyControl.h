#ifndef JOY_CONTROL_H
#define JOY_CONTROL_H

// Own includes
#include <plane_detection_ros/control/JoyStructure.h>

// ROS includes 
#include <sensor_msgs/Joy.h>
#include <ros/ros.h>

namespace joy_control
{

	class JoyControl
	{

	public:
		JoyControl();
		virtual ~JoyControl();

		/**
		 * Joystick callback function.
		 */
		void joyCb(const sensor_msgs::JoyConstPtr& message);

		/**
		 * Return the value for current x-position setpoint offset.
		 */
		double getXOffsetManual();

		/**
		 * Return the value for current y-position setpoint offset.
		 */
		double getYOffsetManual();

		/** 
		 * Return the value for current z-position setpoint offset.
		 */
		double getZOffsetManual();

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
		 * Returns scale value for thrust.
		 */
		double getThrustScale();

		/**
		 * Return scale value for yaw.
		 */
		double getYawScale();

		/**
		 * Return JoyControl pointer.
		 */
		JoyControl* getJoyPointer();

		/**
		 * Return current values of joy buttons array
		 */
		const std::vector<int32_t> getJoyButtons();

		/**
		 * Return vector of float values from Joy message.
		 */
		const std::vector<float> getJoyAxes();
		
		/**
		 * Do all the parameter initialization here.
		 */
		virtual void initializeParameters(ros::NodeHandle& nh);

	private:
		
		/** Current Joy message set in the /joy callback function. */
		sensor_msgs::Joy _joyMsg;

		/** Indices - Joy structure */
		std::unique_ptr<joy_struct::ControlIndices> _controlIndices;

		/** Scale weights - Joy structure */
		std::unique_ptr<joy_struct::ScaleWeights> _attitudeScales;

		/** Scale weights - Joy structure */
		std::unique_ptr<joy_struct::ScaleWeights> _positionScales;
	};

}

#endif /** JOY_CONTROL_H */
