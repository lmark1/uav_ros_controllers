#ifndef CARROT_CONTROL_H
#define CARROT_CONTROL_H

#include <plane_detection_ros/control/ControlBase.h>
#include <plane_detection_ros/control/JoyControl.h>

#include <plane_detection_ros/PositionControlParametersConfig.h>

namespace carrot_control 
{

	/**
	 * "Carrot-on-a-Stick" control implementation.
	 * 
	 * Inherits ControlBase and JoyControl as base classes
	 */
	class CarrotControl : 
		public control_base::ControlBase, 
		public joy_control::JoyControl
	{
	public:

		CarrotControl();
		virtual ~CarrotControl();

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
		 */
		void publishVelMv(ros::Publisher& pub);

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
		 * Do all the parameter initialization here.
		 */
		virtual void initializeParameters(ros::NodeHandle& nh) override;

		/**
		 * Callback function used for setting various parameters.
		 */
		void parametersCallback(
				plane_detection_ros::PositionControlParametersConfig& configMsg,
				uint32_t level);

		/**
		 * Set reconfigure parameters in the given config object.
		 */
		void setReconfigureParameters(
			plane_detection_ros::PositionControlParametersConfig& config);

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
		void updateCarrotX();

		/**
		 * Update y component of carrot position setpoint with the given value.
		 * 
		 * @param - y carrot offset
		 */
		void updateCarrotY(double y);
		void updateCarrotY();

		/**
		 * Update z component of carrot position setpoint with the given value.
		 * 
		 * @param - z carrot offset
		 */
		void updateCarrotZ(double z);
		void updateCarrotZ();

		/**
		 * Return distance to carrot.
		 */
		double distanceToCarrot();

		/**
		 * Return pointer to the CarrotControl object.
		 */
		CarrotControl* getCarrotPointer();

	private:

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
	};	

}

#endif /** CARROT_CONTROL_H */