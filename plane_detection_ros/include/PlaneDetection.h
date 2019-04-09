/*
 * PlaneDetection.h
 *
 *  Created on: Apr 8, 2019
 *      Author: Lovro Markovic
 */

#ifndef PLANE_DETECTION_H
#define PLANE_DETECTION_H

#include <sensor_msgs/PointCloud2.h>
#include <string>

/**
 * This class is used for detecting planes from PointCloud ROS messages.
 */
class PlaneDetection {
public:
	PlaneDetection();
	virtual ~PlaneDetection();

	void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr&);

private:

	const double DISTANCE_TRESHOLD = 0.01;
	const bool ENABLE_OPTIMIZATION = false;
	const std::string FRAME_ID = "leddar";

};

#endif /* PLANE_DETECTION_H */
