#ifndef CARROT_CONTROL_H
#define CARROT_CONTROL_H

#include <plane_detection_ros/control/ControlBase.h>
#include <plane_detection_ros/control/JoyControl.h>

#include <plane_detection_ros/PositionControlParametersConfig.h>

namespace carrot_control 
{
	/** Name of dynamic reconfigure node. */
	#define CARROT_DYN_RECONF "position_config"

	/**
	 * Run default Carrot Control algorithm.
	 * 
	 * @param cc - Reference to CarrotControl object
	 * @param nh - Given NodeHandle
	 */
	void carrot_control::runDefault(carrot_control::CarrotControl& cc, ros::NodeHandle& nh);

	/**
	 * "Carrot-on-a-Stick" control implementation.
	 * It uses cascade od PID controllers to calculate roll / pitch / yaw / thrust,
	 * commands from desired position setpoint.
	 * 
	 * Inherits ControlBase and JoyControl as base classes
	 */
	class CarrotControl : 
		public control_base::ControlBase, 
		public joy_control::JoyControl
	{
	public:

		/**
 		 * Default constructor. Used for reading ROS parameters and initalizing 
		 * private variables.
		 */
		CarrotControl(ros::NodeHandle& nh);
		virtual ~CarrotControl();

		/**
		 * Publish carrot position setpoint as a Vector3 ROS message.
		 */
		void publishPosSp();

		/**
		 * Publish carrot velocity setpoint as a Vector3 ROS message.
		 */ 
		void publishVelSp();

		/**
		 * Publish local position mesured value as a Vector3 ROS message.
		 */
		void publishPosMv();

		/**
		 * Publish measured local velocity value as a Vector3 ROS message.
		 */
		void publishVelMv();

		/**
		 * Set carrot position to given position values.
		 * 
		 * @param x - new carrot x value
		 * @param y - new carrot y value
		 * @param z - new carrot z value
		 */
		void setCarrotPosition(double x, double y, double z); 

		/**
		 * Reset all position PIDs.
		 */
		void resetPositionPID();
		
		/**
		 * Reset all velociy PIDs.
		 */
		void resetVelocityPID();

		/**
		 * Calculate new attitude and thrust setpoint.
		 * 
		 * @oaram dt - Discretization time
		 */
		void calculateAttThrustSp(double dt);

		/**
		 * Update carrot position setpoint, with default joystick offset values.
		 */
		void updateCarrot();

		/**
		 * Update x component of carrot position setpoint with the given value.
		 * 
		 * @param - x carrot offset
		 */
		void updateCarrotX(double x);

		/**
		 * Update x component of carrot position setpoint with default joystick offset 
		 * value.
		 */
		void updateCarrotX();

		/**
		 * Update y component of carrot position setpoint with the given value.
		 * 
		 * @param - y carrot offset
		 */
		void updateCarrotY(double y);

		/**
		 * Update y component of carrot position setpoint with default joystick offset 
		 * value.
		 */
		void updateCarrotY();

		/**
		 * Update z component of carrot position setpoint with the given value.
		 * 
		 * @param - z carrot offset
		 */
		void updateCarrotZ(double z);

		/**
		 * Update z component of carrot position setpoint with default joystick offset 
		 * value.
		 */
		void updateCarrotZ();

		/**
		 * Return distance to carrot.
		 */
		virtual double distanceToCarrot();

		/**
		 * Return squared distance to carrot along x axis.
		 */
		double carrotDistSquaredX();

		/**
		 * Return square distance to carrot along y axis.
		 */
		double carrotDistSquaredY();

		/**
		 * Return squared distance to carrot along z axis.
		 */
		double carrotDistSquaredZ();

		/**
		 * Callback function used for setting various parameters.
		 */
		void positionParamsCb(
				plane_detection_ros::PositionControlParametersConfig& configMsg,
				uint32_t level);
				
	private:

		/**
		 * Set reconfigure parameters in the given config object.
		 */
		void setPositionReconfigureParams(
			plane_detection_ros::PositionControlParametersConfig& config);

		/**
		 * Do all the parameter initialization here.
		 */
		void initializeParameters(ros::NodeHandle& nh);
	

		/** PID controller for position along the y-axis.*/
		std::unique_ptr<PID> _posYPID;
		
		/** PID controller for velocity along the y-axis */
		std::unique_ptr<PID> _velYPID;

		/** PID controller for position along the x-axis.*/
		std::unique_ptr<PID> _posXPID;
		
		/** PID controller for velocity along the x-axis */
		std::unique_ptr<PID> _velXPID;

		/** PID controller for position along the z-axis.*/
		std::unique_ptr<PID> _posZPID;

		/** PID controller for velocity along the y-axis */
		std::unique_ptr<PID> _velZPID;

		/** Carrot setpoint position array. */
		std::array<double, 3> _carrotPos {0.0, 0.0, 0.0};

		/** Carrot setpoint velocity array. */
		std::array<double, 3> _carrotVel {0.0, 0.0, 0.0};

		/** Value from 0 to 1, hover thrust */
		double _hoverThrust;

		/** Define all Publishers. **/
		ros::Publisher _pubCarrotPositionSp;
		ros::Publisher _pubPositionMv;
		ros::Publisher _pubCarrotVelocitySp;
		ros::Publisher _pubVelocityMv;

		/** Define Dynamic Reconfigure parameters **/
		boost::recursive_mutex _posConfigMutex;
		dynamic_reconfigure::
			Server<plane_detection_ros::PositionControlParametersConfig>
			_posConfigServer {_posConfigMutex, ros::NodeHandle(CARROT_DYN_RECONF)};
		dynamic_reconfigure::
			Server<plane_detection_ros::PositionControlParametersConfig>::CallbackType
			_posParamCallback;
	};	

}

#endif /** CARROT_CONTROL_H */