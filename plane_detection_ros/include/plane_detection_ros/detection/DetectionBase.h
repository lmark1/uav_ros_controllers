/*
 * DetectionBase.h
 *
 *  Created on: Apr 9, 2019
 *      Author: lmark
 */

#ifndef DETECTION_BASE_H
#define DETECTION_BASE_H

// ROS includes
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <plane_detection_ros/PlaneDetectionParametersConfig.h>
#include <dynamic_reconfigure/server.h>

// Own includes
#include <uav_ros_control/KalmanFilter.h>

// PCL includes
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> pcl3d_t;
typedef pcl::ModelCoefficients coef_t;

/**
 * Structure containing basic plane information.
 */
struct PlaneInformation
{
	/** Plane centroid. */
	pcl::PointXYZ planeCentroid;

	/** Current plane parameters. */
	coef_t::Ptr planeParameters;

	/** Last detected plane. */
	pcl3d_t planePointCloud;
};

/**
 * Base class for plane detection object. Used for relaying information
 * between ROS and plane detection algorithm.
 */
class DetectionBase
{

/** Distance when plane is not selected. */
#define NO_PLANE_DETECTED -1

/** Minimium number of PointCloud points that need to be available. */
#define MINIMUM_POINTS 3

/** Maximum time with no measurements - Kalman filter*/
#define MAX_INVALID_TIME 2

public:

	/**
	 * Default contructor. Initializes class variables.
	 */
	DetectionBase();
	~DetectionBase();

	/**
	 * Callback function for PointCloud2 objects.
	 * Remembers the last pointcloud object.
	 */
	void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pclMsg);

	/**
	 * Publishes points containing detected plane on a PointCloud2 topic.
	 */
	void publishPlane(ros::Publisher& pub);

	/**
	 * Publishes nomal vector of a detected plane on a PoseStamped topic.
	 */
	void publishNormal(ros::Publisher& pub);

	/**
	 * Publishes calculted distance to the detected plane on a Float64 topic.
	 */
	void publishDistanceToPlane(ros::Publisher& pub);

	/**
	 * Convert the given PointCloud ROS message to
	 * pcl::PointCloud<pcl:PointXYZ> type.
	 */
	void convertFromROSMsg(const sensor_msgs::PointCloud2&, pcl3d_t&);

	/**
	 * Callback function used for setting various parameters.
	 */
	void parametersCallback(
			plane_detection_ros::PlaneDetectionParametersConfig& configMsg,
			uint32_t level);

	/**
	 * Set reconfigure parameters.
	 * 
	 * @param server dynamic_reconfigure Server object, parametrised by type T.
	 */
	template <class T>
	void setReconfigureParameters(dynamic_reconfigure::
		Server<T>& server)
	{
		ROS_WARN("DetectionBase - Reconfigure parameters called.");
		// No parameters need updating here
	}

private:

	/**
	 * Clear current solution.
	 */
	void clearCurrentSolution();

	/**
	 * Check if ready for detection.
	 */
	bool readyForDetection();

	/**
	 * Check if solution is found.
	 */
	bool solutionFound();

	/** Currently detected plane */
	std::unique_ptr<PlaneInformation> _currPlane;

	/** Currently available PointCloud from the callback function. */
	sensor_msgs::PointCloud2 _currPointCloud;

	/** Converted PointCloud from ROS msg. */
	pcl3d_t _currPcl;

	/** Current distance to plane. */
	double _currDistance;

	/** Frame ID where the lidar is located. */
	std::string _frameID;

	/** Flag indicating a new distance measurement was obtained */
	bool _newDistMeasurement;
};

#endif /* DETECTION_BASE_H */
