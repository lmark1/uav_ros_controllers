/*
 * PlaneDetection.h
 *
 *  Created on: Apr 8, 2019
 *      Author: Lovro Markovic
 */

#ifndef PLANE_DETECTION_H
#define PLANE_DETECTION_H

// Own includes
#include <plane_detection_ros/detection/DetectionBase.h>

// Cpp includes
#include <array>

/**
 * This class is used for plane detection from pcl3d_t objects.
 */
class PlaneDetection : public DetectionBase
{
public:

	/**
	 * Default contructor for PlaneDetection class
	 */
	PlaneDetection();
	virtual ~PlaneDetection();

	/**
	 * Detect plane from the given PointCloud<PointXYZ> object.
	 *
	 * @param inputCloud - Input PointCloud (Containing all the points)
	 * @param outputCloud - Output PointCloud (Containing detected plane points)
	 *
	 * @return Plane model coefficients pointer
	 */
	coef_t::Ptr detectPlane (const pcl3d_t& inputCloud, pcl3d_t& planeCloud);

	/**
	 * Filter given PointCloudusing a BoxFilter. 
	 * Ignore all points outside of specified ranges.
	 *
	 * @param inputCloud - Input Pointcloud
	 */
	void filterPointCloud (pcl3d_t& inputCloud);

	/**
	 * Get centroid point from the given PointCloud
	 *
	 * @param inputCloud
	 */
	pcl::PointXYZ getCentroid(const pcl3d_t& inputCloud);

	/**
	 * Project the plane in Y-Z axis.
	 *
	 * @param coefPtr - given pointer to plane coefficients
	 * @param centroid - given plane centroid
	 */
	void projectPlaneToYZ(coef_t::Ptr coefPtr, const pcl::PointXYZ& centroid);

	/**
	 * Calculate distance from given point to plane.
	 *
	 * @param point - given point
	 * @param coef - given plane coefficients
	 */
	double distanceToPlane(const pcl::PointXYZ& point, const coef_t& coef);

	/**
	 * Do all the necessary steps to perform plane detection algorithm.
	 */
	void doPlaneDetection();

	/**
	 * Callback function used for setting various parameters.
	 */
	virtual void parametersCallback(
			plane_detection_ros::PlaneDetectionParametersConfig& configMsg,
			uint32_t level) override;

	/**
	 * Do all the parameter initialization here.
	 */
	virtual void initializeParameters(ros::NodeHandle& nh) override;

	/**
	 * Set reconfigure parameters in the given config object.
	 */
	virtual void setReconfigureParameters(plane_detection_ros::PlaneDetectionParametersConfig& config) override;

private:

	/**
	 * Convert last recieved PointCloud ROS message to
	 * pcl::PointCloud<pcl:PointXYZ> type.
	 * Do this before performing detection.
	 */
	void convertFromROSMsg(const sensor_msgs::PointCloud2&, pcl3d_t&);

	/**
	 * Print a ROS_INFO message about the current state of parameters.
	 */
	void printROSInfo();

	/** Maximum distance from plane to the plane point. */
	double _distanceThreshold;

	/** Enables optimization of parameters. */
	bool _enableOptimization;

	/** Minimum values for box filter used in PointCloud filtering. */
	std::array<float, 3> _boxFilterMin;

	/** Maximum values for box filter used in PointCloud filtering */
	std::array<float, 3> _boxFilterMax;

};

#endif /* PLANE_DETECTION_H */
