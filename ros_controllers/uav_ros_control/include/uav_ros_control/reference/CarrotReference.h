#ifndef CARROT_REFERENCE_H
#define CARROT_REFERENCE_H

#include <uav_ros_control/reference/JoyControlInput.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>

namespace uav_reference 
{
	/**
	 * "Carrot-on-a-Stick" reference publisher class. Reference is published
	 * with respect to the global coordinate system.
	 */
	class CarrotReference : 
		public uav_reference::JoyControlInput
	{
	public:

		/**
 		 * Default constructor. Used for reading ROS parameters and initalizing 
		 * private variables.
		 */
		CarrotReference(ros::NodeHandle&);
		virtual ~CarrotReference();

		/**
		 * Update carrot position setpoint, with default joystick offset values.
		 * Also update the yaw reference value.
		 */
		void updateCarrot();

		/**
		 * Publish carrot position setpoint as a Vector3 ROS message.
		 */
		void publishCarrotSetpoint();

		/**
		 * Publish true if carrot is active, otherwise false.
		 */
		void publishCarrotActivity();

		/** 
		 * Check if carrot mode is entered. This method will reset carrot position
		 * when entering carrot reference mode for the first time.
		 */
		void updateCarrotStatus();

	private:

		/** 
		 * Initialize class parameters.
		 */
		void initializeParameters();


		/**
		 * Update x and y component of carrot position setpoint with the given value.
		 * 
		 * @param - x carrot offset
		 * @param - y carrot offset
		 */
		void updateCarrotXY(double x, double y);

		/**
		 * Update x component of carrot position setpoint with default joystick offset 
		 * value.
		 */
		void updateCarrotXY();

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
		 * Update carrot Yaw component 
		 */
		void updateCarrotYaw();

		/**
		 * Odometry callback function. Used for extracting UAV yaw rotation.
		 */
		void odomCb(const nav_msgs::OdometryConstPtr&);

		/** Carrot setpoint position array. */
		std::array<double, 3> _carrotPos {0.0, 0.0, 0.0};

		/** UAV current position array. */
		std::array<double, 3> _uavPos {0.0, 0.0, 0.0};

		/** Referent yaw angle. */
		double _carrotYaw = 0;

		/** Current UAV yaw angle */
		double _uavYaw = 0;

		/** True if carrot publishing is enabled otherwise false */
		bool _carrotEnabled = false;

		/** Index used for enabling carrot mode */
		int _carrotEnabledIndex = -1;

		/** Define all Publishers */
		ros::Publisher _pubCarrotPositionSp;
		ros::Publisher _pubCarrotYawSp;
		ros::Publisher _pubUAVYawSp;
		ros::Publisher _pubCarrotActivity;

		/** Define all Subscribers. */
		ros::Subscriber _subOdom;
	};

	/**
	 * Run default Carrot Reference publishing node program.
	 * 
	 * @param cc - Reference to CarrotReference object
	 * @param nh - Given NodeHandle
	 */
	void runDefault(uav_reference::CarrotReference& cc, ros::NodeHandle& nh);
}

#endif /** CARROT_REFERENCE_H */