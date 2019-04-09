/*
 * PlaneDetection.cpp
 *
 *  Created on: Apr 8, 2019
 *      Author: Lovro Markovic
 */

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


// ROS includes
#include <ros/console.h>
#include <pcl_conversions/pcl_conversions.h>

// Own includes
#include "PlaneDetection.h"

PlaneDetection::PlaneDetection()
{
}

PlaneDetection::~PlaneDetection()
{
}

void PlaneDetection::pointCloudCallback(
		const sensor_msgs::PointCloud2::ConstPtr& pclMsg)
{
	// Create a new pcl::PointCloud pointer object
	pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(
			new pcl::PointCloud<pcl::PointXYZ>
	);

	// Convert recieved ROS message to pclCloud
	pcl::fromROSMsg (*pclMsg, *pclCloud);
	ROS_DEBUG("New cloud msg: (%i, %i)\n",
			pclCloud->width, pclCloud->height);

	pcl::ModelCoefficients::Ptr modelCoefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (PlaneDetection::DISTANCE_TRESHOLD);
	seg.setInputCloud (pclCloud);
	seg.segment (*inliers, *modelCoefficients);

}


