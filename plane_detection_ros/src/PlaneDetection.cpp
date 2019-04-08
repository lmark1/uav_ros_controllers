/*
 * PlaneDetection.cpp
 *
 *  Created on: Apr 8, 2019
 *      Author: Lovro Markovic
 */

// PCL includes
#include <pcl-1.8/pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

// ROS includes
#include <ros/console.h>

// Own includes
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
}

