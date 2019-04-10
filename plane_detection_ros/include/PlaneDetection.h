/*
 * PlaneDetection.h
 *
 *  Created on: Apr 8, 2019
 *      Author: Lovro Markovic
 */

#ifndef PLANE_DETECTION_H
#define DETECTION_WRAPPER_H

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>


typedef pcl::PointCloud<pcl::PointXYZ> pcl3d_t;
typedef pcl::ModelCoefficients coef_t;

/**
 * This namespace is used for plane detection from pcl3d_t objects.
 */
namespace plane_detect {

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
	 * Filter given PointCloud. Ignore all points outside of specified ranges.
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
	 * Maximum distance from plane to the plane point.
	 */
	double DISTANCE_TRESHOLD = 0.01;

	/**
	 * Enables optimization of parameters.
	 */
	bool ENABLE_OPTIMIZATION = true;

	/**
	 * Filters given PointCloud along the x - axis.
	 */
	float FILTER_X = 2;

	/**
	 * Filters given PointCloud along the y - axis.
	 */
	float FILTER_Y = 2;

	/**
	 * Filters given PointCloud along the z - axis.
	 */
	float FILTER_Z = 2;
}

#endif /* PLANE_DETECTION_H */
