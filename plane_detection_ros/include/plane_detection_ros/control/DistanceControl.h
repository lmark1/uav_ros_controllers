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
	/** Name of dynamic reconfigure node. */
	#define DIST_DYN_RECONF "distance_config"

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
		/**
		 * Left sequence flight activated.
		 */
		LEFT,

		/**
		 * Right sequence flight activated.
		 */
		RIGHT,

		/**
		 * No sequence flight activated.
		 */
		NONE
	};

class DistanceControl : public carrot_control::CarrotControl {

	public:

		/**
 		 * Default constructor. Used for reading ROS parameters and initalizing 
		 * private variables.
		 * 
		 * @param mode - current distance control mode
		 * @param nh - given ROS node handle
		 */
		DistanceControl(DistanceControlMode mode, ros::NodeHandle& nh);
		virtual ~DistanceControl();

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
		 * Publish attitude setpoint on the given topic.
		 * If in simulation mode, publisher is expected to be Vector3.
		 * If in real mode, publisher is expected to be mavros_msgs::AttitudeTarget.
		 */
		void publishAttSp();

		/**
		 * Return true if in inspection state, otherwise false.
		 */
		bool inInspectionState();

		/**
		 * Return the currently active sequence.
		 */
		Sequence getSequence();

		/**
		 * Calculate distance from carrot, taking account of distance from
		 * the plane instead of x-axis carrot position.
		 */
		virtual double distanceToCarrot() override;

		/**
		 * Checks if sequence target is reached.
		 */
		bool seqTargetReached();

		/**
		 * Perform distance control. Set attitude setpoint according to the 
		 * current distance.
		 */
		void doDistanceControl(double dt);

		/**
		 * Callback function used for setting various parameters.
		 */
		void distParamCb(
				plane_detection_ros::DistanceControlParametersConfig& configMsg,
				uint32_t level);

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
		 * Sequence step callback. Step is applied when in sequence mode.
		 */
		void seqStepCb(const std_msgs::Float64ConstPtr& message);

		/**
		 * Publish various distance control algorithm information.
		 */
		void publishDistanceInfo();

	private:

		/**
		 * Publish current control state.
		 */
		void publishState();

		/**
		 * Publish distance setpoint as a std_msgs::Float64 message.
		 */
		void publishDistSp();
		
		/**
		 * Publish distance velocity setpoint as a Float64 ROS message.
		 */
		void publishDistVelSp();

		/**
		 * Publish true if in sequence state, otherwise false.
		 */
		void publishSequenceState();
		
		/**
		 * Publish distance to carrot.
		 */
		void publishDistanceToCarrot();


		/**
		 * Set reconfigure parameters in the given config object.
		 */
		void setDistReconfigureParams(
			plane_detection_ros::DistanceControlParametersConfig& config);	
	
		/**
		 * Initialize parameters.
		 */
		void initializeParameters(ros::NodeHandle& nh);

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

		/** Current sequence step. */
		double _sequenceStep;

		/** Define all ROS subscribers. **/
		ros::Subscriber _subDistance;
		ros::Subscriber _subDistanceVelocity;
		ros::Subscriber _subPlaneNormal;
		ros::Subscriber _subSequenceStep;

		/** Define all ROS publishers. **/
		ros::Publisher _pubControlState;
		ros::Publisher _pubDistanceSp;
		ros::Publisher _pubDistanceVelocitySp;
		ros::Publisher _pubCarrotDistance;
		ros::Publisher _pubSequenceEnabled;

		/** Define Dynamic Reconfigure parameters **/
		boost::recursive_mutex _distConfigMutex;
		dynamic_reconfigure::
			Server<plane_detection_ros::DistanceControlParametersConfig>
			_distConfigServer {_distConfigMutex, ros::NodeHandle(DIST_DYN_RECONF)};
		dynamic_reconfigure::
			Server<plane_detection_ros::DistanceControlParametersConfig>::CallbackType
			_distParamCallback;
	};


	/**
	 * Run default Carrot Control algorithm.
	 * TODO: This does not work - fix
	 * 
	 * @param cc - Reference to CarrotControl object
	 * @param nh - Given NodeHandle
	 * @param simMode - true if simulation mode is enabled, otherwise false
	 */
	void runDefault(
		dist_control::DistanceControl& cc, ros::NodeHandle& nh, bool simMode);

}

#endif /* DISTANCE_CONTROL_H */