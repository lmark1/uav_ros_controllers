#ifndef GEO_FENCE_H
#define GEO_FENCE_H

#include <ros/ros.h>
#include <uav_ros_control/reference/Global2Local.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include "yaml-cpp/yaml.h"
#include <vector>
#include <stdbool.h>


namespace uav_reference
{
	class GeoFence
	{
	public: 

		GeoFence(ros::NodeHandle&, std::string filename);
		virtual ~GeoFence();

	private:

		void referenceCb(const trajectory_msgs::MultiDOFJointTrajectoryPointConstPtr& msg);
		bool checkInside(geometry_msgs::Vector3 current);
		int isLeft(geometry_msgs::Vector3 P0, geometry_msgs::Vector3 P1, geometry_msgs::Vector3 P2);

		Global2Local _global_to_local;  // Object for converting GPS points
		trajectory_msgs::MultiDOFJointTrajectoryPoint _last_valid_position;  // Last received position within fence
		std::vector<geometry_msgs::Vector3> _vertices;  // GPS points definining fence polygon
		double _max_z;  // Maximum allowed flying height

		ros::Publisher _pub;
		ros::Subscriber _sub;

	};

	void runDefault(uav_reference::GeoFence& gf, ros::NodeHandle& nh);
}
#endif
