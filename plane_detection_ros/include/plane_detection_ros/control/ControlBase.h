#ifndef CONTROL_BASE_H
#define CONTROL_BASE_H

// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <uav_ros_control/PID.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

// Own includes
#include <plane_detection_ros/control/JoyStructure.h>

//Cpp includes
#include <array>

namespace control_base
{
	#define MASK_IGNORE_RPY_RATE 7
	#define MASK_IGNORE_RP_RATE 3

	/**
	 * This class is used for defining Control subscribers and publishers.
	 */
	class ControlBase {
	public:

		/**
		 * Default constructor. Used for reading ROS parameters and initalizing private variables.
		 */
		ControlBase();
		virtual ~ControlBase();

		/**
		 * IMU callback function for realistic control mode. 
		 * Calculates UAV yaw.
		 */
		void imuCbReal(const sensor_msgs::ImuConstPtr& message);

		/**
		 * Odometry callback function for simulation control mode.
		 */
		void odomCbSim(const nav_msgs::OdometryConstPtr& message);

		/**
		 * Odometry callback function for real control mode.
		 */
		void odomCbReal(const nav_msgs::OdometryConstPtr& message);

		/**
		 * Return the current UAV yaw angle.
		 */
		double getUAVYaw();

		/**
		 * Return constant reference to the current position.
		 */
		const std::array<double, 3>& getCurrPosition();

		/**
		 * Return constant reference to the current velocity.
		 */
		const std::array<double, 3>& getCurrVelocity();

		/**
		 * Calculates yaw angle from given quaternion components.
		 */
		double calculateYaw(double qx, double qy, double qz, double qw);

		/**
		 * Publish UAV attitude setpoint message of type mav_msgs::RollPitchYawRateThrust
		 * 
		 * @param pub 			- attitude setpoint publisher
		 * @param attThrustSp 	- attitude, thrust setpoint array
		 * @param yawRateSp  	- yaw rate setpoint
		 */
		void publishAttitudeSim(ros::Publisher& pub, const std::array<double, 4>& attThrustSp, double yawRateSp);

		/**
		 * Publish currently set UAV attitude setpoint message of type mav_msgs::RollPitchYawRateThrust.
		 * This method will put yaw angle in currently set attitude setpoint to yaw rate in the message.
		 * 
		 * @param pub			- attitude setpoint publisher
		 * @param thrustScale	- scaling factor for thrust
		 */
		void publishAttitudeSim(ros::Publisher& pub, double thrustScale);

		/**
		 * Publish UAV attitude setpoint message of type mavros_msgs::AttitudeTarget
		 * 
		 * @param pub 			- attitude setpoint publisher
		 * @param attThrustSp 	- attitude, thrust setpoint array
		 * @param yawRateSp  	- yaw rate setpoint
		 * @param typeMask 		- Byte mask used in mavros_msgs::AttitudeTarget message
		 */
		void publishAttitudeReal(ros::Publisher& pub, const std::array<double, 4>& attThrustSp, double yawRateSp, int typeMask);

		/**
		 * Publish UAV attitude setpoint message of type mavros_msgs::AttitudeTarget.
		 * This method will put yaw angle in currently set attitude setpoint to yaw rate in the message.
		 * Type mask will not ignore the yaw rate.
		 * 
		 * @param pub	- attitude setpoint publisher
		 */
		void publishAttitudeReal(ros::Publisher& pub);

		/**
		 * Set roll, pitch, yaw attitude setpoint.
		 * 
		 * @param roll 	- Roll angle setpoint
		 * @param pitch - Pitch angle setpoint
		 * @param yaw 	- Yaw angle setpoint
		 */
		void setAttitudeSp(const double roll, const double pitch, const double yaw);

		/**
		 * Override pitch attitude setpoint.
		 * 
		 * @param pitch - new pitch setpoint
		 */
		void overridePitch(const double pitch);

		/**
		 * Override roll attitude setpoint.
		 *
		 * @param roll - new roll setpoint.
		 */
		void overrideRoll(const double roll);

		/**
		 * Override yaw attitude setpoint.
		 *
		 * @param yaw - new yaw setpoint.
		 */
		void overrideYaw(const double yaw);

		/**
		 * Publish setpoint euler angles as a Vector3 ROS message.
		 */
		void publishEulerSp(ros::Publisher& pub);

		/**
		 * Set thrust setpoint.
		 * 
		 * @param thrust - New thrust setpoint
		 */
		void setThrustSp(const double thrust);

		/**
		 * Return attitude thrust setpoint array;
		 */
		const std::array<double, 4>& getAttThrustSp();

		/**
		 * Return pointer to the control base class.
		 */
		ControlBase* getBasePointer();

	private:

		/**
		 * Update local position.
		 */
		void rotateVector(const double x, const double y, const double z, std::array<double, 3>& vector);

		/** Current local position vector. */
		std::array<double, 3> _currentPosition {0.0, 0.0, 0.0};

		/** Current local velocity vector. */
		std::array<double, 3> _currentVelocity {0.0, 0.0, 0.0};

		/** Attitude setpoint array. */
		std::array<double, 4> _attThrustSp {0.0, 0.0, 0.0, 0.0};

		/** Current UAV yaw angle. */
		double _uavYaw = 0;
	};

}

#endif /* CONTROL_BASE_H */
