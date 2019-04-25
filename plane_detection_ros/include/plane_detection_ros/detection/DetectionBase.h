/*
 * DetectionBase.h
 *
 *  Created on: Apr 9, 2019
 *      Author: lmark
 */

#ifndef DETECTION_BASE_H
#define DETECTION_BASE_H

// PCL includes
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

// ROS includes
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <plane_detection_ros/PlaneDetectionParametersConfig.h>
#include <dynamic_reconfigure/server.h>

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
	coef_t::Ptr planeParameters {new coef_t};

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

public:

	/**
	 * Default contructor. Initializes class variables.
	 */
	DetectionBase();
	virtual ~DetectionBase();

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
	 * Clear current solution.
	 */
	void clearCurrentSolution();

	/**
	 * Check if given PointCloud is ready for detection.
	 */
	bool readyForDetection(const pcl3d_t&);

	/**
	 * Check if solution is found.
	 */
	bool solutionFound();

	/**
	 * Returns plane information reference
	 */
	PlaneInformation& getPlaneRef();

	/**
	 * Return current PointCloud2 ROS message
	 */
	const sensor_msgs::PointCloud2& getPointCloudROS();

	/**
	 * Set current distance to the given value.
	 */
	void setCurrentDistance(double newDistance);
	
	/**
	 * Get current measured distance.
	 */
	double getCurrentDistance();

	/**
	 * Check if new measurement is available.
	 */
	bool newMeasurementReady();

	/**
	 * Reset flag signaling new measurement is ready.
	 */
	void resetNewMeasurementFlag();

	/**
	 * Do all the parameter initialization here.
	 */
	virtual void initializeParameters(ros::NodeHandle& nh);

	/**
	 * Callback function used for setting various parameters.
	 */
	virtual void parametersCallback(
			plane_detection_ros::PlaneDetectionParametersConfig& configMsg,
			uint32_t level);

	/**
	 * Set reconfigure parameters in the given config object.
	 */
	virtual void setReconfigureParameters(plane_detection_ros::PlaneDetectionParametersConfig& config);

private:

	/** Currently detected plane */
	std::unique_ptr<PlaneInformation> _currPlane;

	/** Currently available PointCloud from the callback function. */
	sensor_msgs::PointCloud2 _currPointCloud;

	/** Frame ID where the lidar is located. */
	std::string _frameID;

	/** Current distance to plane. */
	double _currDistance;

	/** Flag indicating a new PointCloud2 ROS message was obtained */
	bool _newPointCloud;
};

#endif /* DETECTION_BASE_H */
