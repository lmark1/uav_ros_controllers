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

/**
 * This class is used for plane detection from pcl3d_t objects.
 */
class PlaneDetection {

public:

	PlaneDetection ();
	virtual ~PlaneDetection ();

	/**
	 * Detect plane from the given PointCloud<PointXYZ> object.
	 *
	 * @inputCloud - Input PointCloude (Containing all the points)
	 * @outputCloud - Output Pointcloud (Containing detected plane points)
	 */
	void detectPlane (const pcl3d_t& inputCloud, pcl3d_t& planeCloud);

private:

	// Define some constants
	const double DISTANCE_TRESHOLD = 0.01;
	const bool ENABLE_OPTIMIZATION = false;

};

#endif /* PLANE_DETECTION_H */
