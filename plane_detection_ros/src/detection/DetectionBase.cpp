/*
 * DetectionWrapper.h
 *
 *  Created on: Apr 9, 2019
 *      Author: lmark
 */

// ROS includes
#include <ros/console.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <plane_detection_ros/PlaneDetectionParametersConfig.h>
#include <tf2/LinearMath/Quaternion.h>

// Cpp includes
#include <vector>
#include <math.h>

// Own includes
#include <plane_detection_ros/detection/DetectionBase.h>

DetectionBase::DetectionBase():
	_currPlane (new PlaneInformation),
	_currDistance (-1),
	_frameID ("default"),
	_newDistMeasurement (false)
{
	_currPlane->planeParameters.operator* = new coef_t();
	_currPlane->planeParameters->values = std::vector<float> (4, 0.0);
	
	_currPlane->planeCentroid.x = 0.0;
	_currPlane->planeCentroid.y = 0.0;
	_currPlane->planeCentroid.z = 0.0;	
}

DetectionBase::~DetectionBase()
{
}

void DetectionBase::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pclMsg)
{
	_currPointCloud = *pclMsg;
	_newDistMeasurement = true;
}

void DetectionBase::parametersCallback(
		plane_detection_ros::PlaneDetectionParametersConfig& configMsg,
		uint32_t level)
{
	ROS_WARN("Hello from DetetionBase reconfigure - no parameters updating here.");
	// No parameters need updating here
}

void DetectionBase::publishPlane(ros::Publisher& pub)
{
	// Construct a new ROS message
	std_msgs::Header header;
	header.stamp 	= ros::Time::now();
	header.frame_id = _frameID;

	// Copy points
	sensor_msgs::PointCloud2 outputMessage;
	pcl::toROSMsg(_currPlane->planePointCloud, outputMessage);
	outputMessage.header = header;

	// Publish
	pub.publish(outputMessage);
}

void DetectionBase::publishNormal(ros::Publisher& pub)
{
	// Construct a new ROS message
	std_msgs::Header header;
	header.stamp 	= ros::Time::now();
	header.frame_id = _frameID;

	geometry_msgs::PoseStamped outputMessage;
	outputMessage.header = header;
	outputMessage.pose.position.x = _currPlane->planeCentroid.x;
	outputMessage.pose.position.y = _currPlane->planeCentroid.y;
	outputMessage.pose.position.z = _currPlane->planeCentroid.z;

	tf2::Quaternion myQuaternion;
	double yaw = atan2(
		_currPlane->planeParameters->values[1],
		_currPlane->planeParameters->values[0]);
	ROS_DEBUG("Normal angle is %f degrees.", yaw * 180/ M_PI);
	myQuaternion.setRPY(0, 0, yaw);
	outputMessage.pose.orientation.x = myQuaternion.x();
	outputMessage.pose.orientation.y = myQuaternion.y();
	outputMessage.pose.orientation.z = myQuaternion.z();
	outputMessage.pose.orientation.w = myQuaternion.w();

	pub.publish(outputMessage);
}

void DetectionBase::publishDistanceToPlane(ros::Publisher& pub)
{
	std_msgs::Float64 outputMessage;
	outputMessage.data = _currDistance;
	pub.publish(outputMessage);
}

void DetectionBase::convertFromROSMsg(
	const sensor_msgs::PointCloud2& input, pcl3d_t& output)
{
	if (input.data.size() == 0)
	{
		ROS_WARN("PointCloud2 message not received, canceling conversion.");
		return;
	}

	// Convert received ROS message to pclCloud
	pcl::fromROSMsg (input, output);
	ROS_DEBUG("Conversion from PointCloud2 to pcl successful");
}

bool DetectionBase::solutionFound()
{
	return _currPlane->planePointCloud.size() > 0 &&
		_currPlane->planeParameters->values.size() == 4;
}

bool DetectionBase::readyForDetection()
{
	return _currPcl.size() > MINIMUM_POINTS;
}

void DetectionBase::clearCurrentSolution()
{
	_currPcl.clear();
	_currPlane->planePointCloud.clear();
	_currPlane->planeParameters->values = std::vector<float> (4, 0.0);
}