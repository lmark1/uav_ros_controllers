//
// Created by robert on 20. 09. 2019..
//

#include <uav_ros_control/reference/VisualServo.h>
#include <math.h>

namespace uav_reference {

VisualServo::VisualServo(ros::NodeHandle& nh) {
  // Define Publishers
  _pubNewSetpoint =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("position_hold/trajectory", 1);

  // Define Subscribers
  _subOdom =
      nh.subscribe("odometry", 1, &uav_reference::VisualServo::odomCb, this);
  _subImu =
      nh.subscribe("imu", 1, &uav_reference::VisualServo::imuCb, this);
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
  _dDistance = 0.0;
}

VisualServo::~VisualServo() {}

bool uav_reference::VisualServo::startVisualServoServiceCb(std_srvs::Empty::Request &request,
                                                           std_srvs::Empty::Response &response) {
  if (!_visualServoEnabled) {
    ROS_INFO("UAV VisualServo - enabling visual servo.");
    _visualServoEnabled = true;
    _yaw_error_integrator = 0.0;
  }
  else {
    ROS_INFO("UAV VisualServo - disabling visual servo.");
    _visualServoEnabled = false;
  }

  return true;
}

void VisualServo::odomCb(const nav_msgs::OdometryConstPtr& odom) {
  _uavPos[0] = odom->pose.pose.position.x;
  _uavPos[1] = odom->pose.pose.position.y;
  _uavPos[2] = odom->pose.pose.position.z;

  if (!ros::param::get("visual_servo_node/use_imu_instead_of_odom", _use_imu)) {
    _use_imu = false;
  }
  if (!_use_imu) {
    _qx = odom->pose.pose.orientation.x;
    _qy = odom->pose.pose.orientation.y;
    _qz = odom->pose.pose.orientation.z;
    _qw = odom->pose.pose.orientation.w;

    _uavYaw = atan2(2 * (_qw * _qz + _qx * _qy), 1.0 - 2.0 * (_qx * _qx + _qy * _qy));
  }
}

void VisualServo::imuCb(const sensor_msgs::ImuConstPtr& imu) {
  if (!ros::param::get("visual_servo_node/use_imu_instead_of_odom", _use_imu)) {
    _use_imu = false;
  }
  if (_use_imu) {
    _qx = imu->orientation.x;
    _qy = imu->orientation.y;
    _qz = imu->orientation.z;
    _qw = imu->orientation.w;
  }
}

void VisualServo::xErrorCb(const std_msgs::Float32 &data) {
  if (!ros::param::get("visual_servo_node/offset_x", _offset_x)) {
    _offset_x = 0.0;
  }
  if (!ros::param::get("visual_servo_node/deadzone_x", _deadzone_x)) {
    _deadzone_x = 0.0;
  }
  if (data.data) _dx = data.data - _offset_x;
  else _dx = 0.0;

  if (abs(_dx) < _deadzone_x) _dx = 0;
}

void VisualServo::yErrorCb(const std_msgs::Float32 &data) {
  if(!ros::param::get("visual_servo_node/offset_y", _offset_y)) {
    _offset_y = 0.0;
  }
  if (!ros::param::get("visual_servo_node/deadzone_y", _deadzone_y)) {
    _deadzone_y = 0.0;
  }

  if(data.data) _dy = data.data - _offset_y;
  else _dy = 0.0;

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
  if (abs(_dYaw) > _yaw_error_integrator_deadzone) {
    _yaw_error_integrator += _dYaw * _yaw_error_integrator_gain;
  }
  _yaw_error_integrator = std::min(_yaw_error_integrator, _yaw_error_integrator_clamp);
  _yaw_error_integrator = std::max(_yaw_error_integrator, -_yaw_error_integrator_clamp);
}

void VisualServo::updateSetpoint() {

  // Define the parameters depending on the scenario.
  // E.g. during the brick laying scenario the yaw, z and distance gain should be zero.

  if (!ros::param::get("visual_servo_node/gain_dx", _gain_dx)) {
    _gain_dx = 0.0;
  }

  if (!ros::param::get("visual_servo_node/gain_dy", _gain_dy)) {
    _gain_dy = 0.0;
  }

  if (!ros::param::get("visual_servo_node/gain_dz", _gain_dz)) {
    _gain_dz = 0.0;
  }

  if (!ros::param::get("visual_servo_node/gain_dYaw", _gain_dYaw)) {
    _gain_dYaw = 0.0;
  }

  if (!ros::param::get("visual_servo_node/gain_dDistance", _gain_dDistance)) {
    _gain_dDistance = 0.0;
  }

  if (!ros::param::get("visual_servo_node/move_saturation", _move_saturation)) {
    _move_saturation = 1.0;
  }

  if (!ros::param::get("visual_servo_node/coordinate_frame_yaw_difference", _coordinate_frame_yaw_difference)) {
    _coordinate_frame_yaw_difference = 0.0;
  }

  if (!ros::param::get("visual_servo_node/yaw_error_integrator_gain", _yaw_error_integrator_gain)) {
    _yaw_error_integrator_gain = 0.0;
  }

  if (!ros::param::get("visual_servo_node/yaw_error_integrator_clamp", _yaw_error_integrator_clamp)) {
    _yaw_error_integrator_clamp = M_PI;
  }

  if (!ros::param::get("visual_servo_node/yaw_error_integrator_deadzone", _yaw_error_integrator_deadzone)) {
    _yaw_error_integrator_deadzone = 0.0;
  }

  double move_forward = -_dy * _gain_dy + _dDistance * _gain_dDistance;
  double move_left = -_dx * _gain_dx;
  double move_up = _dz * _gain_dz;

  if (move_forward > _move_saturation) move_forward = _move_saturation;
  if (move_forward < -_move_saturation) move_forward = -_move_saturation;
  if (move_left > _move_saturation) move_left = _move_saturation;
  if (move_left < -_move_saturation) move_left = -_move_saturation;
  if (move_up > _move_saturation) move_up = _move_saturation;
  if (move_up < -_move_saturation) move_up = -_move_saturation;

  _setpointPosition[0] = _uavPos[0] + move_forward * cos(_uavYaw + _coordinate_frame_yaw_difference);
  _setpointPosition[0] -= move_left * sin(_uavYaw + _coordinate_frame_yaw_difference);
  _setpointPosition[1] = _uavPos[1] + move_forward * sin(_uavYaw + _coordinate_frame_yaw_difference);
  _setpointPosition[1] += move_left * cos(_uavYaw + _coordinate_frame_yaw_difference);
  _setpointPosition[2] = _uavPos[2] + move_up;

  _setpointYaw = _uavYaw + _dYaw * _gain_dYaw + _yaw_error_integrator;

  //ROS_WARN("\n _dx: %f, gain_x: %f", _dx, _gain_dx);
  ROS_WARN("\n _dy: %f, gain_y: %f", _dy, _gain_dy);
  //ROS_WARN("\n _dz: %f, gain_z: %f", _dz, _gain_dz);
  ROS_WARN("\n _dD: %f, gain_D: %f", _dDistance, _gain_dDistance);
  //ROS_WARN("\n _dyaw: %f, gain_yaw: %f", _dYaw, _gain_dYaw);
  ROS_WARN("\n_uavYaw: %f\nforward: %f\nleft:  %f\n", _uavYaw, move_forward, move_left);
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

  ROS_WARN("New setpoint published\n\tx: %.2f -> %.2f \n\ty: %.2f -> %.2f\n\tz: %.2f -> %.2f \n\tYaw: %f -> %f\n\t integrator: %f",
      _uavPos[0], _setpointPosition[0], _uavPos[1], _setpointPosition[1], _uavPos[2], _setpointPosition[2],
      _uavYaw, _setpointYaw, _yaw_error_integrator);
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