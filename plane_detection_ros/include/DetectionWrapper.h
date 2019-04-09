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

// Own includes
#include "PlaneDetection.h"

/**
 * Wrapper class for plane detection object. Used for relaying information
 * between ROS and internal PlaneDetection object.
 */
class DetectionWrapper
{
public:

	DetectionWrapper():
		_planeDetection (new PlaneDetection)
	{
	}

	~DetectionWrapper()
	{
	}

	/**
	 * Returns detection frequency.
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
		_currentPointcloud = *pclMsg;
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

		ROS_DEBUG("Found plane with %i points", this->_currentPlane.size());
		pub.publish(outputMessage);
	}

	/**
	 * Make a call to PlaneDetection::detectPlane() method.
	 */
	void doPlaneDetection()
	{
		if (_currentPointcloud.data.size() == 0)
		{
			ROS_WARN("No PointCloud received, canceling detection.");
			return;
		}

		// Convert received ROS message to pclCloud
		pcl3d_t::Ptr pclCloud {new pcl3d_t};
		pcl::fromROSMsg (_currentPointcloud, *pclCloud);

		// Do the plane detection
		_planeDetection->detectPlane(*pclCloud, _currentPlane);
	}

private:

	std::unique_ptr<PlaneDetection> _planeDetection;
	sensor_msgs::PointCloud2 _currentPointcloud;
	pcl3d_t _currentPlane;

	const std::string FRAME_ID = "leddar";
	const double RATE = 10.0;
};

#endif /* DETECTION_WRAPPER_H */
