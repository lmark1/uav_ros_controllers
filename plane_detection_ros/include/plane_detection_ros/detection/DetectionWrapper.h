/*
 * DetectionWrapper.h
 *
 *  Created on: Apr 9, 2019
 *      Author: lmark
 */

#ifndef DETECTION_WRAPPER_H
#define DETECTION_WRAPPER_H

// ROS includes
#include <ros/console.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <plane_detection_ros/PlaneDetectionParametersConfig.h>
#include <tf2/LinearMath/Quaternion.h>

#include <vector>
#include <math.h>

// Own includes
#include "plane_detection_ros/detection/PlaneDetection.h"
#include <uav_ros_control/KalmanFilter.h>

/** Distance when plane is not selected. */
#define NO_PLANE_DETECTED -1

/** Minimium number of PointCloud points that need to be available. */
#define MINIMUM_POINTS 3

/** Maximum time with no measurements - Kalman filter*/
#define MAX_INVALID_TIME 2

/**
 * Wrapper class for plane detection object. Used for relaying information
 * between ROS and internal PlaneDetection object.
 */
class DetectionWrapper
{
public:

	/**
	 * Default contructor. Initializes class variables.
	 *
	 * @param framID - Given LIDAR sensor frame
	 */
	DetectionWrapper(std::string frameID, double noiseMv, double noisePos, double noiseVel):
		_currPlaneParams (new coef_t),
		_currDistance(-1),
		_newDistMeasurement(false),
		_filteredDistance(-1),
		_timeInvalid(0),
		_kalmanInitialized(false),
		_kalmanFilter(new KalmanFilter),
		FRAME_ID(frameID)
	{
		ROS_INFO("DetectionWrapper - frame_id: %s", frameID.c_str());
		ROS_INFO("Setting kalman parameters: noise_mv=%.2f noise_pose=%.2f noise_vel=%.2f",
			noiseMv, noisePos, noiseVel);
		
		_currCentroid.x = 0.0;
		_currCentroid.y = 0.0;
		_currCentroid.z = 0.0;
		_currPlaneParams->values = std::vector<float> (4, 0.0);

		// Initialize Kalman filter parameters
		_kalmanFilter->setMeasureNoise(noiseMv);
		_kalmanFilter->setPositionNoise(noisePos);
		_kalmanFilter->setVelocityNoise(noiseVel);
	}

	~DetectionWrapper()
	{
	}

	/**
	 * Callback function for PointCloud2 objects.
	 * Remembers the last pointcloud object.
	 */
	void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pclMsg)
	{
		_currPointCloud = *pclMsg;
		_newDistMeasurement = true;
	}

	/**
	 * Callback function used for setting various parameters.
	 */
	void parametersCallback(
			plane_detection_ros::PlaneDetectionParametersConfig& configMsg,
			uint32_t level)
	{
		ROS_WARN("Hello from Re-configure callback.");
		plane_detect::DISTANCE_TRESHOLD = configMsg.dist_tresh;
		plane_detect::ENABLE_OPTIMIZATION = configMsg.param_opt;
		plane_detect::FILTER_MIN_X = configMsg.min_lim_x;
		plane_detect::FILTER_X = configMsg.lim_x;
		plane_detect::FILTER_Y = configMsg.lim_y;
		plane_detect::FILTER_Z = configMsg.lim_z;

		_kalmanInitialized = configMsg.init_kalman;
		_kalmanFilter->setMeasureNoise(configMsg.noise_mv);
		_kalmanFilter->setPositionNoise(configMsg.noise_pos);
		_kalmanFilter->setVelocityNoise(configMsg.noise_vel);
	}

	/**
	 * Set reconfigure parameters.
	 * 
	 * @param server dynamic_reconfigure Server object, parametrised by type T.
	 */
	template <class T>
	void setReconfigureParameters(dynamic_reconfigure::
		Server<T>& server)
	{
		plane_detection_ros::PlaneDetectionParametersConfig configMsg;
		configMsg.dist_tresh = plane_detect::DISTANCE_TRESHOLD;
		configMsg.param_opt = plane_detect::ENABLE_OPTIMIZATION;
		configMsg.lim_x = plane_detect::FILTER_X;
		configMsg.lim_y = plane_detect::FILTER_Y;
		configMsg.lim_z = plane_detect::FILTER_Z;
		configMsg.min_lim_x = plane_detect::FILTER_MIN_X;
		configMsg.init_kalman = _kalmanInitialized;
		configMsg.noise_mv = _kalmanFilter->getMesaureNoise();
		configMsg.noise_pos = _kalmanFilter->getPositionNoise();
		configMsg.noise_vel = _kalmanFilter->getVelocityNoise();
		server.updateConfig(configMsg);
	}

	/**
	 * Publish plane on the given PointCloud2 publisher reference.
	 */
	void publishPlane(ros::Publisher& pub)
	{
		// Construct a new ROS message
		std_msgs::Header header;
		header.stamp 	= ros::Time::now();
		header.frame_id = FRAME_ID;

		// Copy points
		sensor_msgs::PointCloud2 outputMessage;
		pcl::toROSMsg(_currPlaneCloud, outputMessage);
		outputMessage.header = header;

		// Publish
		pub.publish(outputMessage);
	}

	/**
	 * Publish plane normal as a PoseStamped message.
	 */
	void publishNormal(ros::Publisher& pub)
	{
		// Construct a new ROS message
		std_msgs::Header header;
		header.stamp 	= ros::Time::now();
		header.frame_id = FRAME_ID;

		geometry_msgs::PoseStamped outputMessage;
		outputMessage.header = header;
		outputMessage.pose.position.x = _currCentroid.x;
		outputMessage.pose.position.y = _currCentroid.y;
		outputMessage.pose.position.z = _currCentroid.z;

		tf2::Quaternion myQuaternion;
		double yaw = atan2(
				_currPlaneParams->values[1],
				_currPlaneParams->values[0]);
		ROS_DEBUG("Normal angle is %f degrees.", yaw * 180/ M_PI);
		myQuaternion.setRPY(0, 0, yaw);
		outputMessage.pose.orientation.x = myQuaternion.x();
		outputMessage.pose.orientation.y = myQuaternion.y();
		outputMessage.pose.orientation.z = myQuaternion.z();
		outputMessage.pose.orientation.w = myQuaternion.w();

		pub.publish(outputMessage);
	}

	/**
	 * Publish distance to plane as a Float64 message.
	 */
	void publishDistanceToPlane(ros::Publisher& pub)
	{
		std_msgs::Float64 outputMessage;
		outputMessage.data = _currDistance;
		pub.publish(outputMessage);
	}

	void publishFilteredDistance(ros::Publisher& pub)
	{
		std_msgs::Float64 outputMessage;
		outputMessage.data = _filteredDistance;
		pub.publish(outputMessage);
	}

	/**
	 * Convert last recieved PointCloud ROS message to
	 * pcl::PointCloud<pcl:PointXYZ> type.
	 * Do this before performing detection.
	 */
	void convertFromROSMsg()
	{
		if (_currPointCloud.data.size() == 0)
		{
			ROS_WARN("PointCloud2 message not received, canceling conversion.");
			return;
		}

		// Convert received ROS message to pclCloud
		pcl::fromROSMsg (_currPointCloud, _currPcl);
		ROS_DEBUG("Conversion from PointCloud2 to pcl successful");
	}

	/**
	 * Make a polite(!) call to plane_detect::detectPlane() method.
	 */
	void doPlaneDetection()
	{
		if (!readyForDetection())
		{
			ROS_WARN("Canceling detection, insufficient points.");
			clearCurrentSolution();
			return;
		}

		// Do the plane detection
		_currPlaneParams = plane_detect::detectPlane(
				_currPcl, _currPlaneCloud);

		// Check if solution is found
		if (!solutionFound())
		{
			ROS_WARN("No solution found");
			return;
		}

		_currCentroid = plane_detect::getCentroid(_currPlaneCloud);
		plane_detect::projectPlaneToYZ(_currPlaneParams, _currCentroid);

		ROS_DEBUG("Found plane with %lu points", _currPlaneCloud.size());
	}

	/**
	 * PointCloud filtering.
	 */
	void doCloudFiltering()
	{
		plane_detect::filterPointCloud(_currPcl);
	}

	/**
	 * Calcaulte distance to the plane from the UAV.
	 */
	void calculateDistanceToPlane()
	{
		if (solutionFound())
		{
			_currDistance = plane_detect::distanceToPlane(
					pcl::PointXYZ {0,  0, 0}, *_currPlaneParams);
		}
		else
		{
			_currDistance = NO_PLANE_DETECTED;
		}
	}

	/**
	 * Filters current distance using the Kalman filter.
	 *
	 * @param dt - Filter discretization time
	 */
	void filterCurrentDistance(double dt)
	{
		// Reset filtered distance if filter is not initialized
		if (!_kalmanInitialized)
		{
			_filteredDistance = NO_PLANE_DETECTED;
			_timeInvalid = 0;
		}

		// Check if initialization failed
		if (!_kalmanInitialized && _currDistance < 0)
		{
			ROS_WARN("KalmanFilter - Failed to initialize");
			return;
		}

		// Check if initialization should take place
		if (!_kalmanInitialized && _currDistance >= 0)
		{
			_kalmanInitialized = true;
			_kalmanFilter->initializePosition(_currDistance);
			ROS_WARN("KalmanFilter - Initialized.");
		}

		// Do model update
		_kalmanFilter->modelUpdate(dt);

		// If initialized, but invalid reading, do only model update
		if (_newDistMeasurement && _currDistance > 0)
		{
			ROS_INFO("KalmanFilter - New measurement! update called");
			_kalmanFilter->measureUpdate(_currDistance);
			_newDistMeasurement = false;
			_timeInvalid = 0;
		}
		else
		{
			// Increase time invalid
			ROS_WARN("KalmanFilter - doing only model update");
			_timeInvalid += dt;
			ROS_FATAL("KalmanFilter - Time without update: %.2f", _timeInvalid);
		}

		// Check if invalid time reached maximum
		if (_timeInvalid > MAX_INVALID_TIME)
		{
			_kalmanInitialized = false;
			_timeInvalid = 0;
			_filteredDistance = NO_PLANE_DETECTED;
			ROS_FATAL("KalmanFilter - Max invalid time reached.");
			return;
		}

		// Get kalman filter position
		_filteredDistance = _kalmanFilter->getPosition();
	}

private:

	/**
	 * Check if solution is found.
	 */
	bool solutionFound()
	{
		return _currPlaneCloud.size() > 0 &&
				_currPlaneParams->values.size() == 4;
	}

	/**
	 * Check if ready for detection.
	 */
	bool readyForDetection()
	{
		return _currPcl.size() > MINIMUM_POINTS;
	}

	/**
	 * Clear current solution.
	 */
	void clearCurrentSolution()
	{
		_currPcl.clear();
		_currPlaneCloud.clear();
		_currPlaneParams->values = std::vector<float> (4, 0.0);
	}

	/** Kalman filter object. */
	std::unique_ptr<KalmanFilter> _kalmanFilter;

	/** Flag signaling that kalman filter is initialized. */
	bool _kalmanInitialized;

	/** Time passed while measurements are invalid. */
	double _timeInvalid;

	/** Plane centroid. */
	pcl::PointXYZ _currCentroid;

	/** Current plane parameters. */
	coef_t::Ptr _currPlaneParams;

	/** Currently available PointCloud from the callback function. */
	sensor_msgs::PointCloud2 _currPointCloud;

	/** Converted PointCloud from ROS msg. */
	pcl3d_t _currPcl;

	/** Last detected plane. */
	pcl3d_t _currPlaneCloud;

	/** Current distance to plane. */
	double _currDistance;

	/** Flag indicating a new distance measurement was obtained */
	bool _newDistMeasurement;

	/** Filtered distance. */
	double _filteredDistance;

	/** Frame ID where the lidar is located. */
	std::string FRAME_ID;
};

#endif /* DETECTION_WRAPPER_H */
