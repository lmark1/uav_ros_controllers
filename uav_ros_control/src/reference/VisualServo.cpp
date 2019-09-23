//
// Created by robert on 20. 09. 2019..
//

#include <uav_ros_control/reference/VisualServo.h>

namespace uav_reference {

VisualServo::VisualServo(ros::NodeHandle& nh) {
  // Define Publishers
  _pubNewSetpoint =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("position_hold/trajectory", 1);

  // Define Subscribers
  _subOdom =
      nh.subscribe("odometry", 1, &uav_reference::VisualServo::odomCb, this);
  _subXError =
      nh.subscribe("x_error", 1, &uav_reference::VisualServo::xErrorCb, this);
  _subYError =
      nh.subscribe("y_error", 1, &uav_reference::VisualServo::yErrorCb, this);
  _subZError =
      nh.subscribe("z_error", 1, &uav_reference::VisualServo::zErrorCb, this);
  _subYawError =
      nh.subscribe("yaw_error", 1, &uav_reference::VisualServo::yawErrorCb, this);
  _subPitchError =
      nh.subscribe("pitch_error", 1, &uav_reference::VisualServo::pitchErrorCb, this);

  _serviceStartVisualServo = nh.advertiseService(
      "visual_servo",
      &uav_reference::VisualServo::startVisualServoServiceCb,
      this);

  _new_point.transforms = std::vector<geometry_msgs::Transform>(1);
  _new_point.velocities = std::vector<geometry_msgs::Twist>(1);
  _new_point.accelerations = std::vector<geometry_msgs::Twist>(1);
}

VisualServo::~VisualServo() {}

bool uav_reference::VisualServo::startVisualServoServiceCb(std_srvs::Empty::Request &request,
                                                           std_srvs::Empty::Response &response) {
  if (!_visualServoEnabled) {
    ROS_WARN("UAV VisualServo - enabling visual servo.");
    _visualServoEnabled = true;
  }
  else {
    ROS_WARN("UAV VisualServo - disabling visual servo.");
    _visualServoEnabled = false;
  }

  return true;
}

void VisualServo::odomCb(const nav_msgs::OdometryConstPtr& odom) {
  _uavPos[0] = odom->pose.pose.position.x;
  _uavPos[1] = odom->pose.pose.position.y;
  _uavPos[2] = odom->pose.pose.position.z;

  double q0, q1, q2, q3;
  q0 = odom->pose.pose.orientation.x;
  q1 = odom->pose.pose.orientation.y;
  q2 = odom->pose.pose.orientation.z;
  q3 = odom->pose.pose.orientation.w;

  _uavYaw = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
}

void VisualServo::xErrorCb(const std_msgs::Float32 &data) {
  if (!ros::param::get("offset_x", _offset_x)) {
    _offset_x = 0.0;
  }
  if (!ros::param::get("deadzone_x", _deadzone_x)) {
    _deadzone_x = 0.0;
  }
  _dx = data.data - _offset_x;
  if (abs(_dx) < _deadzone_x) _dx = 0;
}

void VisualServo::yErrorCb(const std_msgs::Float32 &data) {
  if(!ros::param::get("offset_y", _offset_y)) {
    _offset_y = 0.0;
  }
  if (!ros::param::get("deadzone_y", _deadzone_y)) {
    _deadzone_y = 0.0;
  }
  _dy = data.data - _offset_y;
  if (abs(_dy) < _deadzone_y) _dy = 0;
}

void VisualServo::zErrorCb(const std_msgs::Float32 &data) {
  _dz = data.data;
}

void VisualServo::pitchErrorCb(const std_msgs::Float32 &data) {
  _dz = data.data;
}

void VisualServo::yawErrorCb(const std_msgs::Float32 &data) {
  _dYaw = data.data;
}

void VisualServo::updateSetpoint() {

  // Define the parameters depending on the scenario.
  // E.g. during the brick laying scenario the yaw, z and distance gain should be zero.

  if (!ros::param::get("gain_dx", _gain_dx)) {
    _gain_dx = 0.0;
  }

  if (!ros::param::get("gain_dy", _gain_dy)) {
    _gain_dy = 0.0;
  }

  if (!ros::param::get("gain_dz", _gain_dz)) {
    _gain_dz = 0.0;
  }

  if (!ros::param::get("gain_dYaw", _gain_dYaw)) {
    _gain_dYaw = 0.0;
  }

  if (!ros::param::get("gain_dDistance", _gain_dDistance)) {
    _gain_dDistance = 0.0;
  }

  double move_forward = -_dy * _gain_dy + _dDistance * _gain_dDistance;
  double move_left = -_dx * _gain_dx;
  double move_up = _dz * _gain_dz;

  _setpointPosition[0] = _uavPos[0] + move_forward * cos(_uavYaw);
  _setpointPosition[0] -= move_left * sin(_uavYaw);
  _setpointPosition[1] = _uavPos[1] + move_forward * sin(_uavYaw);
  _setpointPosition[1] += move_left * cos(_uavYaw);
  _setpointPosition[2] = _uavPos[2] + move_up;

  _setpointYaw = _uavYaw + _dYaw * _gain_dYaw;
}

void VisualServo::publishNewSetpoint() {

  tf2::Quaternion q;
  q.setEulerZYX(_setpointYaw, 0.0, 0.0);

  _new_point.transforms[0].translation.x = _setpointPosition[0];
  _new_point.transforms[0].translation.y = _setpointPosition[1];
  _new_point.transforms[0].translation.z = _setpointPosition[2];
  _new_point.transforms[0].rotation.x = q.getX();
  _new_point.transforms[0].rotation.y = q.getY();
  _new_point.transforms[0].rotation.z = q.getZ();
  _new_point.transforms[0].rotation.w = q.getW();

  _pubNewSetpoint.publish(_new_point);
}

bool VisualServo::isVisualServoEnabled() {
  return _visualServoEnabled;
}

void runDefault(VisualServo& visualServoRefObj, ros::NodeHandle& nh) {
  double rate = 50;
  ros::Rate loopRate(rate);

  while (ros::ok()) {
    ros::spinOnce();
    if (visualServoRefObj.isVisualServoEnabled()) {
      visualServoRefObj.updateSetpoint();
      visualServoRefObj.publishNewSetpoint();
    }
    loopRate.sleep();
  }
}
} // namespace uav_reference