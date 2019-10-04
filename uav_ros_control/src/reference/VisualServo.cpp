//
// Created by robert on 20. 09. 2019..
//

#include <uav_ros_control/reference/VisualServo.h>
#include <math.h>

// Define all parameter paths here
#define VS_P_GAIN_X_PARAM          "reference/p_gain_x"
#define VS_I_GAIN_X_PARAM          "reference/i_gain_x"
#define VS_D_GAIN_X_PARAM          "reference/d_gain_x"
#define VS_I_CLAMP_X_PARAM         "reference/i_clamp_x"
#define VS_OFFSET_X_1_PARAM        "reference/offset_x_1"
#define VS_OFFSET_X_2_PARAM        "reference/offset_x_2"
#define VS_DEADZONE_X_PARAM        "reference/deadzone_x"
#define VS_LANDING_RANGE_X_PARAM   "reference/landing_range_x"

#define VS_P_GAIN_Y_PARAM          "reference/p_gain_y"
#define VS_I_GAIN_Y_PARAM          "reference/p_gain_y"
#define VS_D_GAIN_Y_PARAM          "reference/d_gain_y"
#define VS_I_CLAMP_Y_PARAM         "reference/i_clamp_y"
#define VS_OFFSET_Y_1_PARAM        "reference/offset_y_1"
#define VS_OFFSET_Y_2_PARAM        "reference/offset_y_2"
#define VS_DEADZONE_Y_PARAM        "reference/deadzone_y"
#define VS_LANDING_RANGE_Y_PARAM   "reference/landing_range_y"

#define VS_P_GAIN_Z_PARAM          "reference/p_gain_z"
#define VS_I_GAIN_Z_PARAM          "reference/p_gain_z"
#define VS_D_GAIN_Z_PARAM          "reference/d_gain_z"
#define VS_I_CLAMP_Z_PARAM         "reference/i_clamp_z"
#define VS_OFFSET_Z_1_PARAM        "reference/offset_z_1"
#define VS_OFFSET_Z_2_PARAM        "reference/offset_z_2"
#define VS_DEADZONE_Z_PARAM        "reference/deadzone_z"
#define VS_LANDING_RANGE_Z_PARAM   "reference/landing_range_z"

#define VS_P_GAIN_YAW_PARAM        "reference/p_gain_yaw"
#define VS_I_GAIN_YAW_PARAM        "reference/i_gain_yaw"
#define VS_I_CLAMP_YAW_PARAM       "reference/i_clamp_yaw"
#define VS_I_DEADZONE_YAW_PARAM    "reference/i_deadzone_yaw"
#define VS_D_GAIN_YAW_PARAM        "reference/d_gain_yaw"
#define VS_LANDING_RANGE_YAW_PARAM "reference/lending_range_yaw"

#define VS_P_GAIN_DIST_PARAM       "reference/p_gain_dist"
#define VS_I_GAIN_DIST_PARAM       "reference/i_gain_dist"
#define VS_D_GAIN_DIST_PARAM       "reference/d_gain_dist"
#define VS_I_CLAMP_DIST_PARAM      "reference/i_clamp_dist"
#define VS_DEADZONE_DIST_PARAM     "reference/deadzone_dist"

#define VS_MOVE_SATURATION_PARAM   "reference/move_saturation"
#define VS_YAW_DIFFERENCE_PARAM    "reference/yaw_difference"

#define VS_LANDING_SPEED_PARAM     "reference/landing_speed"
#define VS_IS_BRICK_LAYING_PARAM   "reference/is_brick_laying"

namespace uav_reference {

VisualServo::VisualServo(ros::NodeHandle& nh) {
  // Initialize class parameters
  initializeParameters(nh);

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

  // Setup dynamic reconfigure server

  uav_ros_control::VisualServoParametersConfig VSConfig;
  setVisualServoReconfigureParams(VSConfig);
  _VSConfigServer.updateConfig(VSConfig);
  _VSParamCallback = boost::bind(&VisualServo::visualServoParamsCb, this, _1, _2);
  _VSConfigServer.setCallback(_VSParamCallback);

  _serviceStartVisualServo = nh.advertiseService(
      "visual_servo",
      &uav_reference::VisualServo::startVisualServoServiceCb,
      this);

  _new_point.transforms = std::vector<geometry_msgs::Transform>(1);
  _new_point.velocities = std::vector<geometry_msgs::Twist>(1);
  _new_point.accelerations = std::vector<geometry_msgs::Twist>(1);
  _dDistance = 0.0;
  getParameters();
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

void VisualServo::visualServoParamsCb(uav_ros_control::VisualServoParametersConfig &configMsg,
                                                     uint32_t level) {
  ROS_WARN("VisualServo::parametersCallback");

  _gain_dx = configMsg.k_p_x;
  _gain_dy = configMsg.k_p_y;
  _gain_dz = configMsg.k_p_z;
  _offset_x_1 = configMsg.offset_x_1;
  _offset_x_2 = configMsg.offset_x_2;
  _offset_y_1 = configMsg.offset_y_1;
  _offset_y_2 = configMsg.offset_y_2;
  _deadzone_x = configMsg.deadzone_x;
  _deadzone_y = configMsg.deadzone_y;
  _landing_range_x = configMsg.landing_range_x;
  _landing_range_y = configMsg.landing_range_y;

  _gain_dYaw = configMsg.k_p_yaw;
  _yaw_error_integrator_gain = configMsg.k_i_yaw;
  _yaw_error_integrator_deadzone = configMsg.deadzone_yaw;
  _yaw_error_integrator_clamp = configMsg.clamp_yaw;
  _landing_range_yaw = configMsg.landing_range_yaw;

  _gain_dDistance = configMsg.k_p_dist;

  _move_saturation = configMsg.movement_saturation;
  _coordinate_frame_yaw_difference = configMsg.yaw_difference;
  _visual_servo_shutdown_height = configMsg.shutdown_height;
  _brick_laying_scenario = configMsg.is_bricklaying;
  _landing_speed = configMsg.landing_speed;
}

void VisualServo::setVisualServoReconfigureParams(uav_ros_control::VisualServoParametersConfig &config) {

  ROS_WARN("VisualServo::setVisualServoReconfigureParams");

  config.k_p_x = _gain_dx;
  config.k_p_y = _gain_dy;
  config.k_p_z = _gain_dz;
  config.offset_x_1 = _offset_x_1;
  config.offset_x_2 = _offset_x_2;
  config.offset_y_1 = _offset_y_1;
  config.offset_y_2 = _offset_y_2;
  config.deadzone_x = _deadzone_x;
  config.deadzone_y = _deadzone_y;
  config.landing_range_x = _landing_range_x;
  config.landing_range_y = _landing_range_y;
  config.k_p_yaw = _gain_dYaw;
  config.k_i_yaw = _yaw_error_integrator_gain;
  config.deadzone_yaw = _yaw_error_integrator_deadzone;
  config.clamp_yaw = _yaw_error_integrator_clamp;
  config.k_p_dist = _gain_dDistance;
  config.movement_saturation = _move_saturation;
  config.yaw_difference = _coordinate_frame_yaw_difference;
  config.shutdown_height = _visual_servo_shutdown_height;
  config.is_bricklaying = _brick_laying_scenario;
  config.landing_speed = _landing_speed;
}

void VisualServo::initializeParameters(ros::NodeHandle &nh) {
  ROS_WARN("VisualServo::initializeParameters");
  ROS_INFO("To be implemented after the PIDs");
  getParameters();
}

void VisualServo::odomCb(const nav_msgs::OdometryConstPtr& odom) {
  _uavPos[0] = odom->pose.pose.position.x;
  _uavPos[1] = odom->pose.pose.position.y;
  _uavPos[2] = odom->pose.pose.position.z;

  if (!_use_imu) {
    _qx = odom->pose.pose.orientation.x;
    _qy = odom->pose.pose.orientation.y;
    _qz = odom->pose.pose.orientation.z;
    _qw = odom->pose.pose.orientation.w;

    _uavYaw = atan2(2 * (_qw * _qz + _qx * _qy), 1.0 - 2.0 * (_qx * _qx + _qy * _qy));
  }
}

void VisualServo::imuCb(const sensor_msgs::ImuConstPtr& imu) {
  if (_use_imu) {
    _qx = imu->orientation.x;
    _qy = imu->orientation.y;
    _qz = imu->orientation.z;
    _qw = imu->orientation.w;
  }
}

void VisualServo::xErrorCb(const std_msgs::Float32 &data) {
  double _offset_x_0 = _offset_x_1 + (_offset_x_1 - _offset_x_2);
  _offset_x = (_offset_x_2 - _offset_x_1) * _uavPos[2] + _offset_x_0;
  _offset_x = std::max(_offset_x, 0.0); // avoid negative offsets.

  if (data.data) _dx = data.data - _offset_x;
  else _dx = 0.0;

  if (abs(_dx) < _deadzone_x) _dx = 0;
}

void VisualServo::yErrorCb(const std_msgs::Float32 &data) {
  double _offset_y_0 = _offset_y_1 + (_offset_y_1 - _offset_y_2);
  _offset_y = (_offset_y_2 - _offset_y_1) * _uavPos[2] + _offset_y_0;
  _offset_y = std::max(_offset_y, 0.0);


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

  double move_forward = -_dy * _gain_dy + _dDistance * _gain_dDistance;
  double move_left = -_dx * _gain_dx;
  double move_up = _dz * _gain_dz;

  if (_brick_laying_scenario) {
    if ( abs(_dx) < _landing_range_x && abs(_dy) < _landing_range_y && abs(_dYaw)< _landing_range_yaw) {
      move_up -= _landing_speed;
      if (_uavPos[2] <= _visual_servo_shutdown_height) {
        move_up = 0.1;
        // gas' svjetlo
        // gas' fejs
        // udri
        //_visualServoEnabled = false;

      }
    }
  }

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

  if (_setpointPosition[2] < _visual_servo_shutdown_height){
    _setpointPosition[2] = _visual_servo_shutdown_height;
  }

  _setpointYaw = _uavYaw + _dYaw * _gain_dYaw + _yaw_error_integrator;


  //ROS_WARN("\n _dx: %f, gain_x: %f", _dx, _gain_dx);
  // ROS_WARN("\n _dy: %f, gain_y: %f", _dy, _gain_dy);
  //ROS_WARN("\n _dz: %f, gain_z: %f", _dz, _gain_dz);
  // ROS_WARN("\n _dD: %f, gain_D: %f", _dDistance, _gain_dDistance);
  //ROS_WARN("\n _dyaw: %f, gain_yaw: %f", _dYaw, _gain_dYaw);
  // ROS_WARN("\n_uavYaw: %f\nforward: %f\nleft:  %f\n", _uavYaw, move_forward, move_left);
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

   ROS_WARN("New setpoint published\n\tx: %.2f -> %.2f \n\ty: %.2f -> %.2f\n\tz: %.2f -> %.2f \n\tYaw: %f -> %f\n\tintegrator: %f\n\tdYaw: %f",
      _uavPos[0], _setpointPosition[0], _uavPos[1], _setpointPosition[1], _uavPos[2], _setpointPosition[2],
     _uavYaw, _setpointYaw, _yaw_error_integrator, _dYaw);
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

void VisualServo::getParameters() {

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

  if (!ros::param::get("visual_servo_node/visual_servo_shutdown_height", _visual_servo_shutdown_height)) {
    _visual_servo_shutdown_height = 0.0;
  }

  if (!ros::param::get("visual_servo_node/brick_laying_scenario", _brick_laying_scenario)) {
    _brick_laying_scenario = false;
  }

  if (!ros::param::get("visual_servo_node/landing_speed", _landing_speed)) {
    _landing_speed = 0.0;
  }

  if (!ros::param::get("visual_servo_node/landing_range_x", _landing_range_x)) {
    _landing_range_x = 3*_deadzone_x;
  }

  if (!ros::param::get("visual_servo_node/landing_range_y", _landing_range_y)) {
    _landing_range_y = 3*_deadzone_y;
  }

  if (!ros::param::get("visual_servo_node/landing_range_yaw", _landing_range_yaw)) {
    _landing_range_yaw = 3*_yaw_error_integrator_deadzone;
  }

  if (!ros::param::get("visual_servo_node/use_imu_instead_of_odom", _use_imu)) {
    _use_imu = false;
  }

  if (!ros::param::get("visual_servo_node/offset_x_1", _offset_x_1)) {
    _offset_x_1 = 0.0;
  }

  if (!ros::param::get("visual_servo_node/offset_x_2", _offset_x_2)) {
    _offset_x_2 = 0.0;
  }

  if (!ros::param::get("visual_servo_node/deadzone_x", _deadzone_x)) {
    _deadzone_x = 0.0;
  }

  if (!ros::param::get("visual_servo_node/offset_y_1", _offset_y_1)) {
    _offset_y_1 = 0.0;
  }

  if (!ros::param::get("visual_servo_node/offset_y_2", _offset_y_2)) {
    _offset_y_2 = 0.0;
  }

  if (!ros::param::get("visual_servo_node/deadzone_y", _deadzone_y)) {
    _deadzone_y = 0.0;
  }
}
} // namespace uav_reference