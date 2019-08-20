#include <uav_ros_control/filters/NonlinearFilters.h>
#include <uav_ros_control/reference/CarrotReference.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>

uav_reference::CarrotReference::CarrotReference(ros::NodeHandle& nh) :
	uav_reference::JoyControlInput(nh)
{
	// Define Publishers
	_pubCarrotTrajectorySp = 
		nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("carrot/trajectory", 1);
	_pubCarrotYawSp = 
		nh.advertise<std_msgs::Float64>("carrot/yaw", 1);
	_pubUAVYaw = 
		nh.advertise<std_msgs::Float64>("uav/yaw", 1);

	// Define Subscribers
	_subOdom = 
		nh.subscribe("odometry", 1, &uav_reference::CarrotReference::odomCb, this);
	_subPosHoldRef = 
		nh.subscribe("carrot/traj_point", 1, 
		&uav_reference::CarrotReference::positionRefCb, this);

	// Initialize position hold service
	_servicePoisitionHold = nh.advertiseService(
			"/uav/position_hold",
			&uav_reference::CarrotReference::posHoldServiceCb,
			this);

	initializeParameters();

	// Initialize references
	_carrotPoint.transforms = std::vector<geometry_msgs::Transform>(1);
	_carrotPoint.velocities = std::vector<geometry_msgs::Twist>(1);
	_carrotPoint.accelerations = std::vector<geometry_msgs::Twist>(1);
}

uav_reference::CarrotReference::~CarrotReference()
{ 
}

bool uav_reference::CarrotReference::posHoldServiceCb(std_srvs::Empty::Request& request, 
			std_srvs::Empty::Response& response)
{
	if (!_positionHold)
	{
		ROS_WARN("CarrotControl() - Position hold enabled");
		resetCarrot();	
	}

	return true;
}

void uav_reference::CarrotReference::positionRefCb(
	const trajectory_msgs::MultiDOFJointTrajectoryPointConstPtr& posMsg)
{
	if (_positionHold)
	{
		_carrotPoint = *posMsg;
		_carrotYaw = util::calculateYaw(
			posMsg->transforms[0].rotation.x,
			posMsg->transforms[0].rotation.y,
			posMsg->transforms[0].rotation.z, 
			posMsg->transforms[0].rotation.w);
		ROS_INFO("Trajectory reference received");
	}
	else
	{
		ROS_FATAL("Trajectory reference recieved, but position hold mode is disabled.");
	}
}

void uav_reference::CarrotReference::updateCarrot()
{

	// Disable Position hold if carrot inputs exist
	if (_positionHold && (
		abs(getXOffsetManual()) > 0 || abs(getYOffsetManual()) > 0 || 
		abs(getZOffsetManual()) > 0))
	{
		ROS_WARN("Position hold disabled - resetting carrot position");
		resetCarrot();
		_positionHold = false;
	}

	// Update carrot unless in position hold
	if (!_positionHold)
	{
		updateCarrotXY();
		updateCarrotZ();
		updateCarrotYaw();
	}
}

void uav_reference::CarrotReference::resetCarrot()
{
	_carrotPoint.transforms = std::vector<geometry_msgs::Transform>(1);
	_carrotPoint.velocities = std::vector<geometry_msgs::Twist>(1);
	_carrotPoint.accelerations = std::vector<geometry_msgs::Twist>(1);
	_carrotPoint.transforms[0].translation.x = _uavPos[0];
	_carrotPoint.transforms[0].translation.y = _uavPos[1];
	_carrotPoint.transforms[0].translation.z = _uavPos[2];
	
	// Set carrot orientation
	_carrotYaw = _uavYaw;
	tf2::Quaternion q;
	q.setEuler(_carrotYaw, 0, 0);
	_carrotPoint.transforms[0].rotation.x = q.getX();
	_carrotPoint.transforms[0].rotation.y = q.getY();
	_carrotPoint.transforms[0].rotation.z = q.getZ();
	_carrotPoint.transforms[0].rotation.w = q.getW();
}


void uav_reference::CarrotReference::odomCb(const nav_msgs::OdometryConstPtr& msg)
{
	double qx = msg->pose.pose.orientation.x;
	double qy = msg->pose.pose.orientation.y;
	double qz = msg->pose.pose.orientation.z;
	double qw = msg->pose.pose.orientation.w;

	// Extract UAV yaw
	_uavYaw = util::calculateYaw(qx, qy, qz, qw);

	// Extract UAV position
	_uavPos[0] = msg->pose.pose.position.x;
	_uavPos[1] = msg->pose.pose.position.x;
	_uavPos[2] = msg->pose.pose.position.x;
}

void uav_reference::CarrotReference::updateCarrotYaw()
{
	// Update Carrot yaw angle and wrap to PI
	_carrotYaw += - getYawSpManual();
	_carrotYaw = util::wrapMinMax(_carrotYaw, -M_PI, M_PI);

	tf2::Quaternion q;
	q.setEuler(_carrotYaw, 0, 0);
	_carrotPoint.transforms[0].rotation.x = q.getX();
	_carrotPoint.transforms[0].rotation.y = q.getY();
	_carrotPoint.transforms[0].rotation.z = q.getZ();
	_carrotPoint.transforms[0].rotation.w = q.getW();
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
	// Adjust carrot position w.r.t. the global coordinate system
	_carrotPoint.transforms[0].translation.x += cos(-_uavYaw) * xOff + sin(-_uavYaw) * yOff;
	_carrotPoint.transforms[0].translation.y += cos(-_uavYaw) * yOff - sin(-_uavYaw) * xOff;
}

void uav_reference::CarrotReference::updateCarrotZ(double zOff)
{
	_carrotPoint.transforms[0].translation.z += zOff;
}

void uav_reference::CarrotReference::publishCarrotSetpoint()
{
	tf2::Quaternion q;
	q.setEuler(_carrotYaw, 0, 0);

	// Publish PoseStamped carrot reference
	_pubCarrotTrajectorySp.publish(_carrotPoint);

	// Publish UAV yaw message
	std_msgs::Float64 uavYawMsg;
	uavYawMsg.data = _uavYaw;
	_pubUAVYaw.publish(uavYawMsg);

	// Publish referent yaw message
	std_msgs::Float64 yawRefMsg;
	yawRefMsg.data = _carrotYaw;
	_pubCarrotYawSp.publish(yawRefMsg);
}	

void uav_reference::CarrotReference::initializeParameters()
{
	ROS_WARN("CarrotReference::initializeParameters()");

	ros::NodeHandle nhPrivate("~");
	bool initialized = nhPrivate.getParam("carrot_index", _carrotEnabledIndex);
	ROS_INFO("CarrotReference::initializeParameters() - carrot button enable index is %d",
		_carrotEnabledIndex);
	if (!initialized)
	{
		ROS_FATAL("CarrotReference::initializeParameters() -\
			Carrot enable index not properly initialized.");
		throw std::runtime_error("CarrotReference parameters not properly set.");
	}
}

void uav_reference::CarrotReference::updateCarrotStatus()
{
	// Detect enable button - rising edge
	if (getJoyButtons()[_carrotEnabledIndex] == 1 && !_carrotEnabled)
	{
		_carrotEnabled = true;
		ROS_INFO("CarrotReference::updateCarrotStatus - carrot enabled.");
		resetCarrot();
		ROS_INFO("CarrotReference::updateCarrotStatus - current position and yaw set as carrot reference.");
		return;
	}

	// Detect enable button - falling edge 
	if (getJoyButtons()[_carrotEnabledIndex] == 0 && _carrotEnabled)
	{
		_carrotEnabled = false;
		_positionHold = false;
		ROS_INFO("CarrotRefernce::updateCarrotStatus - carrot disabled.\n");
		return;
	}
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