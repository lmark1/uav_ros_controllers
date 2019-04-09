/*
 * DetectionWrapper.h
 *
 *  Created on: Apr 9, 2019
 *      Author: lmark
 */

#ifndef DETECTION_WRAPPER_H
#define DETECTION_WRAPPER_H

// ROS includes
#include <ros/console.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Header.h>
#include <plane_detection_ros/PlaneDetectionParametersConfig.h>

// Own includes
#include "PlaneDetection.h"

/**
 * Wrapper class for plane detection object. Used for relaying information
 * between ROS and internal PlaneDetection object.
 */
class DetectionWrapper
{
public:

	DetectionWrapper()
	{
	}

	~DetectionWrapper()
	{
	}

	/**
	 * Returns desired detection frequency.
	 */
	double getDetectionRate()
	{
		return this->RATE;
	}

	/**
	 * Callback function for PointCloud2 objects.
	 * Remembers the last pointcloud object.
	 */
	void pointCloudCallback(
			const sensor_msgs::PointCloud2::ConstPtr& pclMsg)
	{
		_currentPointCloud = *pclMsg;
	}

	/**
	 * Callback function used for setting various parameters.
	 */
	void parametersCallback(
			plane_detection_ros::PlaneDetectionParametersConfig& configMsg,
			uint32_t lecvel)
	{
		ROS_WARN("Hello from callbdack");
		plane_detect::DISTANCE_TRESHOLD = configMsg.dist_tresh;
		plane_detect::ENABLE_OPTIMIZATION = configMsg.param_opt;
		plane_detect::FILTER_X = configMsg.lim;
		plane_detect::FILTER_Y = configMsg.lim;
		plane_detect::FILTER_Z = configMsg.lim;
	}

	/**
	 * Publish plane on the given PointCloud2 publisher reference.
	 */
	void publishPlane(ros::Publisher& pub)
	{
		if (this->_currentPlane.size() == 0)
		{
			ROS_WARN("No plane ready for publishing.");
			return;
		}

		// Construct a new ROS message
		std_msgs::Header header;
		header.stamp 	= ros::Time::now();
		header.frame_id = FRAME_ID;

		// Copy points
		sensor_msgs::PointCloud2 outputMessage;
		pcl::toROSMsg(this->_currentPlane, outputMessage);
		outputMessage.header = header;

		// Publish
		pub.publish(outputMessage);
	}

	/**
	 * Convert last recieved PointCloud ROS message to
	 * pcl::PointCloud<pcl:PointXYZ> type.
	 * Do this before performing detection.
	 */
	void convertFromROSMsg()
	{
		if (_currentPointCloud.data.size() == 0)
		{
			ROS_WARN("PointCloud2 message not received, canceling conversion.");
			return;
		}

		// Convert received ROS message to pclCloud
		pcl::fromROSMsg (_currentPointCloud, _convertedPointCloud);
		ROS_DEBUG("Conversion successful");
	}

	/**
	 * Make a polite(!) call to plane_detect::detectPlane() method.
	 */
	void doPlaneDetection()
	{
		if (_convertedPointCloud.size() == 0)
		{
			ROS_WARN("Converted PointCloud empty, canceling detection.");
			return;
		}

		// Do the plane detection
		plane_detect::detectPlane(_convertedPointCloud, _currentPlane);
		ROS_DEBUG("Found plane with %lu points", _currentPlane.size());
	}

	/**
	 * PointCloud filtering.
	 */
	void doCloudFiltering()
	{
		if (_convertedPointCloud.size() == 0)
		{
			ROS_WARN("Converted PointCloud empty, canceling filtering.");
			return;
		}

		plane_detect::filterPointCloud(_convertedPointCloud);
		ROS_DEBUG("Filtering successful");
	}

private:

	/**
	 * Currently available PointCloud from the callback function.
	 */
	sensor_msgs::PointCloud2 _currentPointCloud;

	/**
	 * Converted PointCloud from ROS msg.
	 */
	pcl3d_t _convertedPointCloud;

	/**
	 * Last detected plane.
	 */
	pcl3d_t _currentPlane;

	/**
	 * Frame ID where the lidar is located.
	 */
	std::string FRAME_ID = "leddar";

	/**
	 * Rate of the detection.
	 */
	double RATE = 10.0;
};

#endif /* DETECTION_WRAPPER_H */
