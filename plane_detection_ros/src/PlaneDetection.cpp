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
#include <pcl/segmentation/sac_segmentation.h>

// Own includes
#include "PlaneDetection.h"

PlaneDetection::PlaneDetection()
{
}

PlaneDetection::~PlaneDetection()
{
}

void PlaneDetection::detectPlane(
		const pcl3d_t& currPointCloud, pcl3d_t& planeCloud)
{

	// Create output objects pointers
	pcl::ModelCoefficients::Ptr modelCoefficients {new pcl::ModelCoefficients};
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	// Initialize RANSAC filter
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients (PlaneDetection::ENABLE_OPTIMIZATION);
	seg.setModelType 			(pcl::SACMODEL_PLANE);
	seg.setMethodType 			(pcl::SAC_RANSAC);
	seg.setDistanceThreshold 	(PlaneDetection::DISTANCE_TRESHOLD);
	seg.setInputCloud			(pcl3d_t::ConstPtr {new pcl3d_t(currPointCloud)} );
	seg.segment 				(*inliers, *modelCoefficients);

	// Copy all indices to a new PointCloud
	pcl::copyPointCloud<pcl::PointXYZ>(currPointCloud, inliers->indices,
			planeCloud);
}


