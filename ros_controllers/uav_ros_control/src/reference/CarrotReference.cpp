#include <uav_ros_control/filters/NonlinearFilters.h>
#include <uav_ros_control/reference/CarrotReference.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>

uav_reference::CarrotReference::CarrotReference(ros::NodeHandle& nh) :
	uav_reference::JoyControlInput(nh)
{
	_pubCarrotPositionSp = 
		nh.advertise<geometry_msgs::PoseStamped>("/carrot/position", 1);
	_pubCarrotYawSp = 
		nh.advertise<std_msgs::Float64>("/carrot/yaw", 1);
	_pubUAVYawSp = 
		nh.advertise<std_msgs::Float64>("/uav/yaw", 1);
	_subOdom = 
		nh.subscribe("/odometry", 1, &uav_reference::CarrotReference::odomCb, this);
}

uav_reference::CarrotReference::~CarrotReference()
{ }

void uav_reference::CarrotReference::updateCarrot()
{
	updateCarrotXY();
	updateCarrotZ();
	updateCarrotYaw();
}

void uav_reference::CarrotReference::odomCb(const nav_msgs::OdometryConstPtr& msg)
{
	double qx = msg->pose.pose.orientation.x;
	double qy = msg->pose.pose.orientation.y;
	double qz = msg->pose.pose.orientation.z;
	double qw = msg->pose.pose.orientation.w;

	// Extract UAV yaw
	_uavYaw = atan2(2 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz);

	// Extract UAV position
	_uavPos[0] = msg->pose.pose.position.x;
	_uavPos[1] = msg->pose.pose.position.x;
	_uavPos[2] = msg->pose.pose.position.x;
}

void uav_reference::CarrotReference::updateCarrotYaw()
{
	_carrotYaw += - getYawSpManual();
	_carrotYaw = nonlinear_filters::wrapMinMax(_carrotYaw, -M_PI, M_PI);
}

void uav_reference::CarrotReference::updateCarrotXY()
{
	updateCarrotXY(getXOffsetManual(), getYOffsetManual());
}

void uav_reference::CarrotReference::updateCarrotZ()
{
	updateCarrotZ(getZOffsetManual());
}

void uav_reference::CarrotReference::updateCarrotXY(double xOff, double yOff)
{
	_carrotPos[0] += cos(-_uavYaw) * xOff + sin(-_uavYaw) * yOff;
	_carrotPos[1] += cos(-_uavYaw) * yOff - sin(-_uavYaw) * xOff;
}

void uav_reference::CarrotReference::updateCarrotZ(double zOff)
{
	_carrotPos[2] += zOff;
}

void uav_reference::CarrotReference::publishCarrotSetpoint()
{
	tf2::Quaternion q;
	q.setEuler(_carrotYaw, 0, 0);

	geometry_msgs::PoseStamped msg;
	msg.pose.position.x = _carrotPos[0];
	msg.pose.position.y = _carrotPos[1];
	msg.pose.position.z = _carrotPos[2];
	msg.pose.orientation.x = q.getX();
	msg.pose.orientation.y = q.getY();
	msg.pose.orientation.z = q.getZ();
	msg.pose.orientation.w = q.getW();
	_pubCarrotPositionSp.publish(msg);

	std_msgs::Float64 uavYawMsg;
	uavYawMsg.data = _uavYaw;
	_pubUAVYawSp.publish(uavYawMsg);

	std_msgs::Float64 yawRefMsg;
	yawRefMsg.data = _carrotYaw;
	_pubCarrotYawSp.publish(yawRefMsg);
}	

void uav_reference::CarrotReference::updateCarrotStatus()
{
	// TODO: Implement carrot mode swapping here
}

void uav_reference::runDefault(
	uav_reference::CarrotReference& carrotRefObj, ros::NodeHandle& nh)
{
	double rate = 50;
	ros::Rate loopRate(rate);
	
	while (ros::ok())
	{
		ros::spinOnce();
		carrotRefObj.updateCarrotStatus();
		carrotRefObj.updateCarrot();
		carrotRefObj.publishCarrotSetpoint();
		loopRate.sleep();
	}
}