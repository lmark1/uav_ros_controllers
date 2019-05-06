/*
 * DistanceControl.h
 *
 *  Created on: Apr 13, 2019
 *      Author: lmark
 */

#ifndef DISTANCE_CONTROL_H
#define DISTANCE_CONTROL_H

#include <plane_detection_ros/control/ControlBase.h>
#include <plane_detection_ros/control/CarrotControl.h>
#include <plane_detection_ros/DistanceControlParametersConfig.h>

// ROS Includes
#include <ros/ros.h>

#include <iostream>
#include <array>

namespace dist_control 
{

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

	enum Sequence
	{
		LEFT,
		RIGHT,
		NONE
	};

class DistanceControl : public carrot_control::CarrotControl {

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
		 * Distance callback function.
		 */
		void distanceCb(const std_msgs::Float64ConstPtr& message);

		/**
		 * Distance velocity callback function.
		 */
		void distanceVelCb(const std_msgs::Float64ConstPtr& message);

		/**
		 * Plane normal callback function.
		 */
		void normalCb(const geometry_msgs::PoseStampedConstPtr& message);

		/**
		 * Change the state back to manual if received distance is invalid.
		 * Change the state to inspection mode if appropriate joystick command is
		 * given.
		 */
		void detectStateChange();

		/**
		 * Check for any change in desired seqnce of inspection and perform all
		 * necessary changes.
		 */
		void detectSequenceChange();

		/**
		 * Calculate target attitude setpoint while in manual mode. 
		 * During Manual mode UAV is controlled with direct attitude commands.
		 *
		 * @param dt - Given discretization time.
		 */
		void calculateManualSetpoint(double dt);
		
		/**
		 * Calculate target attitude setpoint while in inspection mode.
		 * During Inspection mode UAV is controlled using "Carrot commands" i.e. 
		 * position control.
		 * 
		 * @param dt - Given discretization time.
		 */
		void calculateInspectionSetpoint(double dt);

		/**
		 * Calculate target attitude setpoint while in a sequence.
		 * When performing a sequence the UAV is moved a fixed amount to either
		 * left or right side. It will hold position for a fixed amount of time
		 * before moving again.
		 */
		void calculateSequenceSetpoint(double dt);

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
		 * Return true if in inspection state, otherwise false.
		 */
		bool inInspectionState();

		/**
		 * Return the currently active sequence.
		 */
		Sequence getSequence();

		/**
		 * Initialize parameters.
		 */
		virtual void initializeParameters(ros::NodeHandle& nh) override;

		/**
		 * Callback function used for setting various parameters.
		 */
		void parametersCallback(
				plane_detection_ros::DistanceControlParametersConfig& configMsg,
				uint32_t level);

		/**
		 * Set reconfigure parameters in the given config object.
		 */
		void setReconfigureParameters(
			plane_detection_ros::DistanceControlParametersConfig& config);
		
	private:
		
		/**
		 * Perform distance control. Set attitude setpoint according to the 
		 * current distance.
		 */
		void doDistanceControl(double dt);

		/**
		 * From the current Joy message determine if left sequence is enabled.
		 */
		bool leftSeqEnbled();

		/**
		 * From the current Joy message determine if right sequence is enabled.
		 */
		bool rightSeqEnabled();

		/**
		 * From the current Joy message determine if inspection if not.
		 */
		bool inspectionEnabled();

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

		/** Current sequence */
		Sequence _currSeq;

		/** Distance PID controller */
		std::unique_ptr<PID> _distancePID;

		/** Distance velocity PID controller */
		std::unique_ptr<PID> _distanceVelPID;
		
		/** Inspection indices for ROS Joy messages */
		std::unique_ptr<joy_struct::InspectionIndices> _inspectIndices;
		
		/** True if inspection state was requested and denied, false otherwise. */
		bool _inspectionRequestFailed;

		/** Current distance measured value. Used both in sim and real mode. */
		double _distanceMeasured;

		/** Currently measured distance velocity. Used both in sim and real mode. */
		double _distanceVelocityMeasured;

		/** Distance setpoint value */
		double _distSp;

		/** Distance velocity setpoint value. */
		double _distVelSp;

		/** Yaw of the plane normal with respect to UAV base frame. */
		double _planeYaw;
	};

}

#endif /* DISTANCE_CONTROL_H */