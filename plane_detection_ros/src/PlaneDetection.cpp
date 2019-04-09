/*
 * PlaneDetection.cpp
 *
 *  Created on: Apr 8, 2019
 *      Author: Lovro Markovic
 */

// PCL includes
#include <pcl/conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/crop_box.h>

// Own includes
#include "PlaneDetection.h"

// Eigen
#include <Eigen/Core>

void plane_detect::detectPlane(
		const pcl3d_t& currPointCloud, pcl3d_t& planeCloud)
{

	// Create output objects pointers
	pcl::ModelCoefficients::Ptr modelCoefficients {new pcl::ModelCoefficients};
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	// Initialize RANSAC filter
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients (plane_detect::ENABLE_OPTIMIZATION);
	seg.setModelType 			(pcl::SACMODEL_PLANE);
	seg.setMethodType 			(pcl::SAC_RANSAC);
	seg.setDistanceThreshold 	(plane_detect::DISTANCE_TRESHOLD);
	seg.setInputCloud			(pcl3d_t::ConstPtr {new pcl3d_t(currPointCloud)});
	seg.segment 				(*inliers, *modelCoefficients);

	// Copy all indices to a new PointCloud
	pcl::copyPointCloud<pcl::PointXYZ>(currPointCloud, inliers->indices,
			planeCloud);
}

void plane_detect::filterPointCloud (pcl3d_t& inputCloud)
{
	pcl::CropBox<pcl::PointXYZ> boxFilter;
	boxFilter.setMin(Eigen::Vector4f {
		- plane_detect::FILTER_X,
		- plane_detect::FILTER_Y,
		- plane_detect::FILTER_Z,
		0
	});
	boxFilter.setMax(Eigen::Vector4f {
		plane_detect::FILTER_X,
		plane_detect::FILTER_Y,
		plane_detect::FILTER_Z,
		0
	});
	boxFilter.setInputCloud (pcl3d_t::ConstPtr {new pcl3d_t(inputCloud)});
	boxFilter.filter (inputCloud);
}
