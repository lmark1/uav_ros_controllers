/*
 * PlaneDetection.h
 *
 *  Created on: Apr 8, 2019
 *      Author: Lovro Markovic
 */

#ifndef PLANE_DETECTION_H
#define PLANE_DETECTION_H

#include <sensor_msgs/PointCloud2.h>

/**
 * This class is used for detecting planes from PointCloud ROS messages.
 */
class PlaneDetection {
public:
	PlaneDetection();
	virtual ~PlaneDetection();

	void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& pointCloud);
};

#endif /* PLANE_DETECTION_H */
