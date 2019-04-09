/*
 * PlaneDetection.cpp
 *
 *  Created on: Apr 8, 2019
 *      Author: Lovro Markovic
 */

// PCL includes
#include <pcl/conversions.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// ROS includes
#include <ros/console.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Header.h>

// Own includes
#include "PlaneDetection.h"

typedef pcl::PointCloud<pcl::PointXYZ> pcl3d_t;

PlaneDetection::PlaneDetection()
{
}

PlaneDetection::~PlaneDetection()
{
}

double PlaneDetection::getDetectionRate()
{
	return this->RATE;
}

void PlaneDetection::pointCloudCallback(
		const sensor_msgs::PointCloud2::ConstPtr& pclMsg)
{
	this->currPointcloud = *pclMsg;
}

void PlaneDetection::detectPlane()
{
	if (this->currPointcloud.data.size() == 0)
	{
		ROS_WARN("No PointCloud received, stopping detection.");
		return;
	}

	// Create a new pcl::PointCloud pointer object
	pcl3d_t::Ptr pclCloud {new pcl3d_t};

	// Convert recieved ROS message to pclCloud
	pcl::fromROSMsg (this->currPointcloud, *pclCloud);

	// Create output objects pointers
	pcl::ModelCoefficients::Ptr modelCoefficients {new pcl::ModelCoefficients};
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	// Initialize RANSAC filter
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients (PlaneDetection::ENABLE_OPTIMIZATION);
	seg.setModelType 			(pcl::SACMODEL_PLANE);
	seg.setMethodType 			(pcl::SAC_RANSAC);
	seg.setDistanceThreshold 	(PlaneDetection::DISTANCE_TRESHOLD);
	seg.setInputCloud			(pcl3d_t::ConstPtr {new pcl3d_t(*pclCloud)} );
	seg.segment 				(*inliers, *modelCoefficients);

	// Check if any solution is found
	if (inliers->indices.size() == 0)
	{
		ROS_DEBUG("No solution found.");
		return;
	}

	// Copy all indices to a new PointCloud
	pcl::copyPointCloud<pcl::PointXYZ>(*pclCloud, inliers->indices,
			this->currentPlane);
}

void PlaneDetection::publishPlane(ros::Publisher& pub)
{
	if (this->currentPlane.size() == 0)
	{
		ROS_WARN("No plane ready for publishing.");
		return;
	}

	// Construct a new ROS message
	std_msgs::Header header;
	header.stamp 	= ros::Time::now();
	header.frame_id = PlaneDetection::FRAME_ID;

	// Copy points
	sensor_msgs::PointCloud2 outputMessage;
	pcl::toROSMsg(this->currentPlane, outputMessage);
	outputMessage.header = header;

	ROS_DEBUG("Found plane with %i points", this->currentPlane.size());
	pub.publish(outputMessage);
}


