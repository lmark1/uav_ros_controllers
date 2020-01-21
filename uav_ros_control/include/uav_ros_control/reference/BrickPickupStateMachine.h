#ifndef VISUAL_SERVO_STATE_MACHINE_H
#define VISUAL_SERVO_STATE_MACHINE_H

#include <std_msgs/Int32.h>
#include <ros/ros.h>
#include <uav_ros_control/BrickPickupStateMachineParametersConfig.h>
#include <uav_ros_control_msgs/GeoBrickApproach.h>
#include <uav_ros_control/filters/Util.h>
#include <iostream>
#include <uav_ros_control/reference/Global2Local.h>
#include <uav_ros_control/reference/TrajectoryGenerator.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/SetBool.h>
#include <color_filter/color.h>

using namespace ros_util;

namespace uav_sm {

enum class BrickPickupStates {
  OFF,
  APPROACH,
  SEARCH,
  ATTEMPT_PICKUP
};

enum class VisualServoState {
  OFF
  // Some other states as well
};

struct BrickPickupStatus {
  BrickPickupStatus() : BrickPickupStatus("red", Eigen::Vector3d(0, 0, 0)) { }
  BrickPickupStatus(const std::string& t_brickColor, const Eigen::Vector3d& t_brickLocal, 
    const BrickPickupStates t_status = BrickPickupStates::OFF) :
      m_brickColor(t_brickColor),
      m_status(t_status), 
      m_localBrick(t_brickLocal) { }

  BrickPickupStates m_status;
  std::string m_brickColor;
  Eigen::Vector3d m_localBrick;
};

typedef uav_ros_control::BrickPickupStateMachineParametersConfig PickupParams;
typedef uav_ros_control_msgs::GeoBrickApproach GeoBrickMsg;
typedef GeoBrickMsg::Request GeoBrickReq;
typedef GeoBrickMsg::Response GeoBrickResp;

class BrickPickupStateMachine {

public:
BrickPickupStateMachine(ros::NodeHandle& t_nh) :
    m_handlerVSSMState(t_nh, "visual_servo_sm/state"),
    m_handlerOdometry(t_nh, "odometry"),
    m_global2Local(t_nh) {
  initialize_parameters(t_nh);
  
  // Initialize service callers
  m_vssmCaller = t_nh.serviceClient
    <std_srvs::SetBool::Request, std_srvs::SetBool::Response>("brick_pickup/local");
  m_chooseColorCaller = t_nh.serviceClient
    <color_filter::color::Request, color_filter::color::Response>("filter_color");

  // Advertise service
  m_serviceBrickPickup = t_nh.advertiseService(
    "brick_pickup/global",
    &uav_sm::BrickPickupStateMachine::brick_pickup_global_cb,
    this);
  
  m_runTimer = t_nh.createTimer(
    ros::Duration(1.0 / getParamOrThrow<double>(t_nh, "brick_pickup/rate")), 
    &uav_sm::BrickPickupStateMachine::run_once, this);
}

private:
void run_once(const ros::TimerEvent& /* unused */) {
  
}

bool brick_pickup_global_cb(GeoBrickReq& request, GeoBrickResp& response) {
  // If we want to disable the global brick pickup
  if (!request.enable) {
    ROS_FATAL("BPSM::brick_pickup_global_cb - brick_pickup/global disabled");
    response.status = false;
    //m_currentState == BrickPickupStates::OFF;
    return true;
  }

  // Enable global brick pickup
  m_currentStatus = BrickPickupStatus(request.brick_color,
    m_global2Local.toLocal(
      request.latitude, request.longitude, request.altitude_relative, true), 
    BrickPickupStates::APPROACH);

  ROS_INFO("Current brick goal: [%.3f, %.3f, %.3f]", 
    m_currentStatus.m_localBrick.x(), m_currentStatus.m_localBrick.y(), 
    m_currentStatus.m_localBrick.z());
  return true;
}

inline const VisualServoState getCurrentVisualServoState() {
  return static_cast<VisualServoState>(m_handlerVSSMState.getData().data);
}

void initialize_parameters(ros::NodeHandle& nh) {
  PickupParams initParams;
  initParams.dummy_param = getParamOrThrow<double>(nh, "brick_pickup/dummy_param");
  m_pickupConfig.reset(new ParamHandler<PickupParams>(initParams, "brick_pickup"));
}

bool toggle_visual_servo_state_machine(bool t_enable = true) {
  std_srvs::SetBool::Request req;
  std_srvs::SetBool::Response resp;
  req.data = t_enable;

  if (!m_vssmCaller.call(req, resp)) {
    ROS_FATAL("BrickPickup - unable to activate VSSM");
    return false;
  }

  // At this point VSSM is successfully toggled, so return true
  return true;
}

bool filter_choose_color(const std::string& t_color) {
  color_filter::color::Request req;
  color_filter::color::Response resp;
  req.color = t_color;

  if (!m_chooseColorCaller.call(req, resp)) {
    ROS_FATAL("BrickPickup - color initialization failed");
    return false;
  }

  // At this point color_initialization is assumed to be finished
  return true;
}

ros::Timer m_runTimer;
Global2Local m_global2Local;
BrickPickupStatus m_currentStatus;
ros::ServiceServer m_serviceBrickPickup;
ros::ServiceClient m_vssmCaller, m_chooseColorCaller;
TopicHandler<std_msgs::Int32> m_handlerVSSMState;
TopicHandler<nav_msgs::Odometry> m_handlerOdometry;
std::unique_ptr<ParamHandler<PickupParams>> m_pickupConfig;
};
}
#endif
