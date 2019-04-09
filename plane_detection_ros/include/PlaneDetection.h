/*
 * PlaneDetection.h
 *
 *  Created on: Apr 8, 2019
 *      Author: Lovro Markovic
 */

#ifndef PLANE_DETECTION_H
#define PLANE_DETECTION_H

#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>

/**
 * This class is used for detecting planes from PointCloud ROS messages.
 */
class PlaneDetection {

public:

	PlaneDetection ();
	virtual ~PlaneDetection ();

	/**
	 * Returns detection frequency.
	 */
	double getDetectionRate ();

	/**
	 * Callback function for PointCloud2 objects.
	 * Remembers the last pointcloud object.
	 */
	void pointCloudCallback (const sensor_msgs::PointCloud2ConstPtr&);

	/**
	 * Detect plane using the last PointCloud2 object obtained from
	 * the callback function.
	 */
	void detectPlane ();

	/**
	 * Publish plane on the given PointCloud2 publisher reference.
	 */
	void publishPlane (ros::Publisher&);

private:

	sensor_msgs::PointCloud2 currPointcloud;
	pcl::PointCloud<pcl::PointXYZ> currentPlane;

	// Define some constants
	const double DISTANCE_TRESHOLD = 0.01;
	const bool ENABLE_OPTIMIZATION = false;
	const std::string FRAME_ID = "leddar";
	const double RATE = 10.0;

};

#endif /* PLANE_DETECTION_H */
