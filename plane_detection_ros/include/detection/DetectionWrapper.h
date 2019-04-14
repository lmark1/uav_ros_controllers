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
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <plane_detection_ros/PlaneDetectionParametersConfig.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

// Own includes
#include "detection/PlaneDetection.h"

/**
 * Wrapper class for plane detection object. Used for relaying information
 * between ROS and internal PlaneDetection object.
 */
class DetectionWrapper
{
public:

	DetectionWrapper():
		_currPlaneParams (new coef_t)
	{
		_currCentroid.x = 0.0;
		_currCentroid.y = 0.0;
		_currCentroid.z = 0.0;
		_currPlaneParams->values = std::vector<float> (4, 0.0);
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
		_currPointCloud = *pclMsg;
	}

	/**
	 * Callback function used for setting various parameters.
	 */
	void parametersCallback(
			plane_detection_ros::PlaneDetectionParametersConfig& configMsg,
			uint32_t level)
	{
		ROS_WARN("Hello from Configure callbdack");
		plane_detect::DISTANCE_TRESHOLD = configMsg.dist_tresh;
		plane_detect::ENABLE_OPTIMIZATION = configMsg.param_opt;
		plane_detect::FILTER_X = configMsg.lim_x;
		plane_detect::FILTER_Y = configMsg.lim_y;
		plane_detect::FILTER_Z = configMsg.lim_z;
	}

	/**
	 * Publish plane on the given PointCloud2 publisher reference.
	 */
	void publishPlane(ros::Publisher& pub)
	{
		// Construct a new ROS message
		std_msgs::Header header;
		header.stamp 	= ros::Time::now();
		header.frame_id = FRAME_ID;

		// Copy points
		sensor_msgs::PointCloud2 outputMessage;
		pcl::toROSMsg(_currPlaneCloud, outputMessage);
		outputMessage.header = header;

		// Publish
		pub.publish(outputMessage);
	}

	void publishNormal(ros::Publisher& pub)
	{
		// Construct a new ROS message
		std_msgs::Header header;
		header.stamp 	= ros::Time::now();
		header.frame_id = FRAME_ID;

		geometry_msgs::PoseStamped outputMessage;
		outputMessage.header = header;
		outputMessage.pose.position.x = _currCentroid.x;
		outputMessage.pose.position.y = _currCentroid.y;
		outputMessage.pose.position.z = _currCentroid.z;

		tf2::Quaternion myQuaternion;
		double yaw = atan2(
				_currPlaneParams->values[1],
				_currPlaneParams->values[0]);
		myQuaternion.setRPY(0, 0, yaw);
		outputMessage.pose.orientation.x = myQuaternion.x();
		outputMessage.pose.orientation.y = myQuaternion.y();
		outputMessage.pose.orientation.z = myQuaternion.z();
		outputMessage.pose.orientation.w = myQuaternion.w();

		pub.publish(outputMessage);
	}

	void publishDistanceToPlane(ros::Publisher& pub)
	{
		std_msgs::Float64 outputMessage;

		// Careful not to publish a faulty solution
		if (solutionFound())
			outputMessage.data = plane_detect::distanceToPlane(
					pcl::PointXYZ {0,  0, 0}, *_currPlaneParams);
		else
			outputMessage.data = NO_PLANE_DETECTED;

		pub.publish(outputMessage);
	}

	/**
	 * Convert last recieved PointCloud ROS message to
	 * pcl::PointCloud<pcl:PointXYZ> type.
	 * Do this before performing detection.
	 */
	void convertFromROSMsg()
	{
		if (_currPointCloud.data.size() == 0)
		{
			ROS_WARN("PointCloud2 message not received, canceling conversion.");
			return;
		}

		// Convert received ROS message to pclCloud
		pcl::fromROSMsg (_currPointCloud, _currPcl);
		ROS_DEBUG("Conversion successful");
	}

	/**
	 * Make a polite(!) call to plane_detect::detectPlane() method.
	 */
	void doPlaneDetection()
	{
		if (!readyForDetection())
		{
			ROS_WARN("Canceling detection, insufficient points.");
			clearCurrentSolution();
			return;
		}

		// Do the plane detection
		_currPlaneParams = plane_detect::detectPlane(
				_currPcl, _currPlaneCloud);

		// Check if solution is found
		if (!solutionFound())
		{
			ROS_WARN("No solution found");
			return;
		}

		_currCentroid = plane_detect::getCentroid(_currPlaneCloud);
		plane_detect::projectPlaneToYZ(_currPlaneParams, _currCentroid);

		ROS_DEBUG("Found plane with %lu points \n", _currPlaneCloud.size());
	}

	/**
	 * PointCloud filtering.
	 */
	void doCloudFiltering()
	{
		plane_detect::filterPointCloud(_currPcl);
		ROS_DEBUG("Filtering successful");
	}

private:

	/**
	 * Check if solution is found.
	 */
	bool solutionFound()
	{
		return _currPlaneCloud.size() > 0 &&
				_currPlaneParams->values.size() == 4;
	}

	/**
	 * Check if ready for detection.
	 */
	bool readyForDetection()
	{
		return _currPcl.size() > MINIMUM_POINTS;
	}

	/**
	 * Clear current solution.
	 */
	void clearCurrentSolution()
	{
		_currPcl.clear();
		_currPlaneCloud.clear();
		_currPlaneParams->values = std::vector<float> (4, 0.0);
	}

	/**
	 * Plane centroid.
	 */
	pcl::PointXYZ _currCentroid;

	/**
	 * Current plane parameters.
	 */
	coef_t::Ptr _currPlaneParams;

	/**
	 * Currently available PointCloud from the callback function.
	 */
	sensor_msgs::PointCloud2 _currPointCloud;

	/**
	 * Converted PointCloud from ROS msg.
	 */
	pcl3d_t _currPcl;

	/**
	 * Last detected plane.
	 */
	pcl3d_t _currPlaneCloud;

	/**
	 * Distance when plane is not selected
	 */
	const double NO_PLANE_DETECTED = -1.0;

	/**
	 * Minimium number of PointCloud points that need to be available.
	 */
	const int MINIMUM_POINTS = 3;

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
