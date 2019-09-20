//
// Created by robert on 20. 09. 2019..
//

#include <uav_ros_control/reference/VisualServo.h>

namespace uav_reference {

VisualServo::VisualServo(ros::NodeHandle &nh) {
  // Define Publishers
  _pubNewSetpoint =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("position_hold/trajectory", 1);

  // Define Subscribers
  _subOdom =
      nh.subscribe("msf_core/odometry", 1, &uav_reference::VisualServo::odomCb, this);
  _subYawError =
      nh.subscribe("color_filter/yaw_err", 1, &uav_reference::VisualServo::yawErrorCb, this);
  _subPitchError =
      nh.subscribe("color_filter/pitch_err", 1, &uav_reference::VisualServo::pitchErrorCb, this);

}

void VisualServo::updateSetpoint() {
  double move_forward = -_dy;
  double move_left = -_dx;

  _setpointPosition[0] = _uavPos[0] + move_forward * cos(_uav_yaw);
  _setpointPosition[0] -= move_left * sin(_uav_yaw);
  _setpointPosition[1] = _uavPos[1] + move_forward * sin(_uav_yaw);
  _setpointPosition[1] += move_left * cos(_uav_yaw);
  _setpointPosition[2] = _uavPos[2];

  // TODO setpoint yaw
}

void VisualServo::publishNewSetpoint() {

}

bool VisualServo::isVisualServoEnabled() {
  return _visualServoEnabled;
}
} // namespace uav_reference