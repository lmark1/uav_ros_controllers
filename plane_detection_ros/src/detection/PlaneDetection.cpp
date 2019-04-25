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
#include <pcl/common/centroid.h>

// Own includes
#include "plane_detection_ros/detection/PlaneDetection.h"

// Eigen
#include <Eigen/Core>

// ROS includes
#include <pcl_conversions/pcl_conversions.h>

PlaneDetection::PlaneDetection():
	_distanceThreshold (0.01),
	_enableOptimization (true),
	_boxFilterMin {0.5, -2, -2},
	_boxFilterMax {2, 2, 2},
	DetectionBase()
{
}

PlaneDetection::~PlaneDetection()
{
}

coef_t::Ptr PlaneDetection::detectPlane(
		const pcl3d_t& currPointCloud, pcl3d_t& planeCloud)
{

	// Create output objects pointers
	coef_t::Ptr modelCoefficients {new pcl::ModelCoefficients};
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	// Initialize RANSAC filter
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients (_enableOptimization);
	seg.setModelType 			(pcl::SACMODEL_PLANE);
	seg.setMethodType 			(pcl::SAC_RANSAC);
	seg.setDistanceThreshold 	(_distanceThreshold);
	seg.setInputCloud			(pcl3d_t::ConstPtr {new pcl3d_t(currPointCloud)});
	seg.segment 				(*inliers, *modelCoefficients);

	// Copy all indices to a new PointCloud
	pcl::copyPointCloud<pcl::PointXYZ>(currPointCloud, inliers->indices,
			planeCloud);

	return modelCoefficients;
}

void PlaneDetection::filterPointCloud (pcl3d_t& inputCloud)
{
	pcl::CropBox<pcl::PointXYZ> boxFilter;
	boxFilter.setMin(Eigen::Vector4f {
		_boxFilterMin[0],
		_boxFilterMin[1],
		_boxFilterMin[2],
		0
	});
	boxFilter.setMax(Eigen::Vector4f {
		_boxFilterMax[0],
		_boxFilterMax[1],
		_boxFilterMax[2],
		0
	});
	boxFilter.setInputCloud (pcl3d_t::ConstPtr {new pcl3d_t(inputCloud)});
	boxFilter.filter (inputCloud);
}

pcl::PointXYZ PlaneDetection::getCentroid(const pcl3d_t& inputCloud)
{
	pcl::CentroidPoint<pcl::PointXYZ> centroidCalc;
	for(auto& point : inputCloud.points)
		centroidCalc.add(point);

	pcl::PointXYZ centroid;
	centroidCalc.get(centroid);

	return centroid;
}

void PlaneDetection::projectPlaneToYZ(
		coef_t::Ptr coefPtr, const pcl::PointXYZ& centroid)
{
	double d = sqrt(
			pow(coefPtr->values[0], 2) + pow(coefPtr->values[1], 2));

	// A component
	coefPtr->values[0] = coefPtr->values[0] / d;

	// B component
	coefPtr->values[1] = coefPtr->values[1] / d;

	// C component
	coefPtr->values[2] = 0.0;

	// D component
	coefPtr->values[3] =
			- coefPtr->values[0] * centroid.x
			- coefPtr->values[1] * centroid.y
			- coefPtr->values[2] * centroid.z;
}

double PlaneDetection::distanceToPlane(
		const pcl::PointXYZ& point, const coef_t& coef)
{
	double num = fabs(
			coef.values[0] * point.x +
			coef.values[1] * point.y +
			coef.values[2] * point.z +
			coef.values[3]);

	double den = sqrt(
			pow(coef.values[0], 2) +
			pow(coef.values[1], 2) +
			pow(coef.values[2], 2) );

	return num / den;
}

void PlaneDetection::doPlaneDetection()
{
	// Convert the PCL message
	pcl3d_t convertedPcl;
	convertFromROSMsg(getPointCloudROS(), convertedPcl);

	// Do the cloud filtering
	filterPointCloud(convertedPcl);

	// Check if ready for detection
	if (!readyForDetection(convertedPcl))
	{
		ROS_WARN("Not ready for detection - cancelling.");
		clearCurrentSolution();
		return;
	}

	// Do the plane detection
	getPlaneRef().planeParameters = detectPlane(convertedPcl, getPlaneRef().planePointCloud);

	// Check if solution is found
	if (!solutionFound())
	{
		ROS_WARN("No solution found");
		clearCurrentSolution();
		return;
	}

	// Calculate centroid and project to YZ plane
	getPlaneRef().planeCentroid = getCentroid(getPlaneRef().planePointCloud);
	projectPlaneToYZ(getPlaneRef().planeParameters, getPlaneRef().planeCentroid);
	ROS_DEBUG("Found plane with %lu points", getPlaneRef().planePointCloud.size());

	// Calculate distance to the found plane
	double newDistance = distanceToPlane(pcl::PointXYZ {0,  0, 0}, *getPlaneRef().planeParameters);
	setCurrentDistance(newDistance);
	ROS_DEBUG("Distance to found plane is %.2f", newDistance);
}

void PlaneDetection::convertFromROSMsg(
	const sensor_msgs::PointCloud2& input, pcl3d_t& output)
{
	// Check if received input message is valid
	if (input.data.size() == 0)
	{
		ROS_WARN("PointCloud2 message not received, canceling conversion.");
		return;
	}

	// Convert received ROS message to pclCloud
	pcl::fromROSMsg (input, output);
	ROS_DEBUG("Conversion from PointCloud2 to pcl successful");
}

void PlaneDetection::initializeParameters(ros::NodeHandle& nh)
{
	DetectionBase::initializeParameters(nh);
	ROS_DEBUG("PlaneDetection::initializeParameters()");
	
	bool initialized = 
		nh.getParam("/detection/lim_x", _boxFilterMax[0]) &&
		nh.getParam("/detection/lim_y", _boxFilterMax[1]) &&
		nh.getParam("/detection/lim_z", _boxFilterMax[2]) &&
		nh.getParam("/detection/min_x", _boxFilterMin[0]) &&
		nh.getParam("/detection/threshold", _distanceThreshold);

	_boxFilterMin[1] = - _boxFilterMax[1];
	_boxFilterMin[2] = - _boxFilterMax[2];

	printROSInfo();

	// Check if PlaneDetection is properly initialized
	if (!initialized)
	{
		ROS_FATAL("PlaneDetection::initializeParameters() - parameter initializeation failed.");
		throw std::invalid_argument("PlaneDetection parameters not properly set.");
	}
}

void PlaneDetection::parametersCallback(
	plane_detection_ros::PlaneDetectionParametersConfig& configMsg,
	uint32_t level)
{
	DetectionBase::parametersCallback(configMsg, level);
	ROS_WARN("Hello from PlaneDetection parameters callback.");
	_boxFilterMax[0] = configMsg.lim_x;
	_boxFilterMax[1] = configMsg.lim_y;
	_boxFilterMax[2] = configMsg.lim_z;
	_boxFilterMin[0] = configMsg.min_lim_x;
	_boxFilterMin[1] = - configMsg.lim_y;
	_boxFilterMin[2] = - configMsg.lim_z;
	_distanceThreshold = configMsg.dist_tresh;
	_enableOptimization = configMsg.param_opt;
	printROSInfo();
}

void PlaneDetection::setReconfigureParameters(plane_detection_ros::PlaneDetectionParametersConfig& config)
{
	DetectionBase::setReconfigureParameters(config);
	ROS_WARN("PlaneDetection - Reconfigure parameters called.");
	config.param_opt = _enableOptimization;
	config.dist_tresh = _distanceThreshold;
	config.lim_x = _boxFilterMax[0];
	config.lim_y = _boxFilterMax[1];
	config.lim_z = _boxFilterMax[2];
	config.min_lim_x = _boxFilterMin[0];
}

void PlaneDetection::printROSInfo()
{
	ROS_INFO("New BoxFilter Max - [%.2f, %.2f, %.2f]",
		_boxFilterMax[0], _boxFilterMax[1], _boxFilterMax[2]);
	ROS_INFO("New BoxFilter Min - [%.2f, %.2f, %.2f]",
		_boxFilterMin[0], _boxFilterMin[1], _boxFilterMin[2]);
	ROS_INFO("New distance threshold: %.2f", _distanceThreshold);
}
