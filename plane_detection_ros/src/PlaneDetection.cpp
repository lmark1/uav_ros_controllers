/*
 * PlaneDetection.cpp
 *
 *  Created on: Apr 8, 2019
 *      Author: Lovro Markovic
 */

#include <ros/console.h>
#include "PlaneDetection.h"

PlaneDetection::PlaneDetection()
{
}

PlaneDetection::~PlaneDetection()
{
}



void PlaneDetection::pointCloudCallback(
		const sensor_msgs::PointCloud2ConstPtr& pointCloud)
{
	ROS_INFO("Hello from callback");
}

