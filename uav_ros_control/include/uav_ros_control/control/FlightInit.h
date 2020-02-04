/*
 * FlightInit.h
 *
 *  Created on: Feb 5, 2020
 *      Author: AnaBatinovic
 */

#ifndef FLIGHT_INIT_H
#define FLIGHT_INIT_H
#include <iostream>
// ROS Includes
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/service_server.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <sensor_msgs/NavSatFix.h>

#include <dynamic_reconfigure/server.h>
#include <uav_ros_control/FlightInitParametersConfig.h>
#include <uav_ros_control/filters/NonlinearFilters.h>

namespace flight_init 
{
/**
 * Name of dynamic reconfigure node.
 */
typedef uav_ros_control::FlightInitParametersConfig fi_param_t;
#define FLIGHT_INIT_DYN_RECONF  	"flight_init_config"
#define PARAM_TIME_FOR_INIT			"flight_init_node/time_for_init"
#define PARAM_RATE					"flight_init_node/rate"
#define PARAM_TAKEOFF_HEIGHT		"flight_init_node/takeoff_height"

class FlightInit 
{
public: 
	/**
	 * Default constructor.
	*/
	FlightInit (ros::NodeHandle& nh)
	{
		initializeParameters(nh);

		// Define subscribers 
		m_subOdometry = nh.subscribe(
			"mavros/global_position/local", 1, 
			&flight_init::FlightInit::odometryCb, this);

		m_subState = nh.subscribe<mavros_msgs::State>(
				"mavros/state", 1, 
				&flight_init::FlightInit::stateCb, this);

		m_subGlobalPosition = nh.subscribe<sensor_msgs::NavSatFix>(
			"mavros/global_position/global", 10, 
			&flight_init::FlightInit::globalPositionCb, this);

		// Setup takeoff service callback
    	m_serviceTakeOff = nh.advertiseService(
			"takeoff", &flight_init::FlightInit::takeOffCb, this);

		m_armingClient = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    	m_setModeClient = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
		m_takeoffClient = nh.serviceClient<mavros_msgs::CommandTOL>
			("mavros/cmd/takeoff");
		// Setup dynamic reconfigure server
		fi_param_t  fiConfig;
		setReconfigureParameters(fiConfig);
		m_fiConfigServer.updateConfig(fiConfig);
		m_fiParamCallback = boost::bind(
			&flight_init::FlightInit::fiParamCb, this, _1, _2);
		m_fiConfigServer.setCallback(m_fiParamCallback);

	}

void initializeParameters(ros::NodeHandle& nh)
{
    ROS_INFO("FlightInit::initializeParameters()");
    bool initialized = 
        nh.getParam(PARAM_TIME_FOR_INIT, m_timeForInit)
		&& nh.getParam(PARAM_TAKEOFF_HEIGHT, m_takeoffHeight)
        && nh.getParam(PARAM_RATE, m_rate);

    ROS_INFO("Node rate: %.2f", m_rate);
    ROS_INFO("Time for initialization: %.2f", m_timeForInit);
	ROS_INFO("Takeoff height: %.2f", m_takeoffHeight);
    if (!initialized)
	{
		ROS_FATAL("FlightInit::initializeParameters() - failed to initialize parameters");
		throw std::runtime_error("FlightInit parameters not properly initialized.");
	}
}

void fiParamCb(fi_param_t& configMsg,uint32_t level)
{
    ROS_WARN("FlightInit::fiParamCb()");
    m_timeForInit = configMsg.time_for_init;
	m_takeoffHeight = configMsg.takeoff_height;

}

void setReconfigureParameters(fi_param_t& configMsg)
{
	ROS_WARN("Hello from setReconfigureParameters");
	configMsg.time_for_init = m_timeForInit;
	configMsg.takeoff_height = m_takeoffHeight;
}

void stateCb(const mavros_msgs::State::ConstPtr& msg)
{
	m_currentState = *msg;
}

void globalPositionCb(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	m_currentGlobalPosition = *msg;
}

bool takeOffCb(
	std_srvs::SetBool::Request& request, 
	std_srvs::SetBool::Response& response)
{

	ROS_INFO("Rosservice called.");
	m_serviceTakeoffCalled = true;
	response.success = true;
	response.message = "Takeoff service called.";
	return true;
}
void odometryCb(const nav_msgs::OdometryConstPtr& msg)
{
	m_currentOdom = *msg;
}

bool arm()
{	
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;
	// Arm throttle
	
	if (m_currentState.mode == "GUIDED" &&
		!m_currentState.armed)
	{
		if (m_armingClient.call(arm_cmd) && arm_cmd.response.success)
		{
			ROS_INFO("Vehicle armed");
			return true;
		}
		return false;
	}
	return false;
}

bool modeGuided()
{
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "GUIDED";
	// Enable guided
	if (m_currentState.mode != "GUIDED")
	{
		if(m_setModeClient.call(offb_set_mode) && offb_set_mode.response.mode_sent)
		{
			std::cout << "STATE: " << m_currentState.mode << std::endl;
			ROS_INFO("GUIDED enabled");
			return true;

		}
		return false;
	}
	return false;
}	

bool takeoff()
{
	double current_yaw = util::calculateYaw(
        m_currentOdom.pose.pose.orientation.x,
        m_currentOdom.pose.pose.orientation.y,
        m_currentOdom.pose.pose.orientation.z,
        m_currentOdom.pose.pose.orientation.w);
	
	mavros_msgs::CommandTOL takeoff;
	takeoff.request.min_pitch = 0;
	takeoff.request.yaw = current_yaw;
	takeoff.request.latitude = m_currentGlobalPosition.latitude;
	takeoff.request.longitude = m_currentGlobalPosition.longitude;
	takeoff.request.altitude = m_currentGlobalPosition.altitude + m_takeoffHeight;

	if (m_currentState.armed)
	{
		if (m_takeoffClient.call(takeoff))
		{
			ROS_INFO("Takeoff!!");
			return true;
		}
		return false;
	}
	return false;
}

void run()
{
    ros::Rate loopRate(m_rate);

	
	// ros::Time last_request = ros::Time::now();
	 // Wait for FCU connection
	ros::Time last_request = ros::Time::now();
	while (ros::ok())
	{
		ros::spinOnce();
		// std::cout << "Offb set mode: " << offb_set_mode.request.custom_mode << std::endl;
		// std::cout << "Current state mode: " << m_currentState.mode << std::endl;
		bool guided = modeGuided();
		bool armed = arm();
		bool go = takeoff();
		if (go)
		{
			ROS_INFO("Mission completed!");
			break;
		}		

		// ROS_INFO("Waiting for connection.");
        loopRate.sleep();
    }
}

private: 

double m_timeForInit, m_takeoffHeight, m_rate;
mavros_msgs::State m_currentState;
sensor_msgs::NavSatFix m_currentGlobalPosition;
nav_msgs::Odometry m_currentOdom;
ros::Subscriber m_subState, m_subGlobalPosition, m_subOdometry;

bool m_serviceTakeoffCalled = false;

ros::ServiceServer m_serviceTakeOff;
ros::ServiceClient m_armingClient, m_setModeClient, m_takeoffClient;
/* Define Dynamic Reconfigure parameters */
boost::recursive_mutex m_fiConfigMutex;
dynamic_reconfigure::Server<fi_param_t>
	m_fiConfigServer {m_fiConfigMutex, ros::NodeHandle(FLIGHT_INIT_DYN_RECONF)};
dynamic_reconfigure::Server<fi_param_t>::CallbackType m_fiParamCallback;

};

}

#endif /* FLIGHT_INIT_H */