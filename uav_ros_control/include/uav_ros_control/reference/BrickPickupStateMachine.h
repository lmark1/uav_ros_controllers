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
#include <std_msgs/Bool.h>

using namespace ros_util;

namespace uav_sm {

enum class BrickPickupStates {
  OFF,
  APPROACH,
  SEARCH,
  ATTEMPT_PICKUP, 
  DROPOFF
};

enum class VisualServoState {
  OFF
  // Some other states as well
};

struct BrickPickupStatus {
  BrickPickupStatus() : BrickPickupStatus("red", Eigen::Vector3d(0, 0, 0)) { }
  BrickPickupStatus(const std::string& t_brickColor, const Eigen::Vector3d& t_brickLocal, 
    const BrickPickupStates t_status = BrickPickupStates::OFF) : // TODO: Set this to default OFF
      m_brickColor(t_brickColor),
      m_status(t_status), 
      m_localBrick(t_brickLocal) { }

  bool isOff() {
    return m_status == BrickPickupStates::OFF;
  }

  bool isSearching() {
    return m_status == BrickPickupStates::SEARCH;
  }

  bool isApproaching() {
    return m_status == BrickPickupStates::APPROACH;
  }

  bool isAttemptingPickup() {
    return m_status == BrickPickupStates::ATTEMPT_PICKUP;
  }

  bool isDropOff() {
    return m_status == BrickPickupStates::DROPOFF;
  }

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
    m_handlerOdometry(t_nh, "/mavros/global_position/local"),
    m_handlerBrickAttached(t_nh, "brick_attached"),
    m_global2Local(t_nh) {
  initialize_parameters(t_nh);
  
  // Initialize publisher
  m_pubPositionHold = t_nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
    "position_hold/trajectory", 1);

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
  
  // Iniitalize timers
  m_runTimer = t_nh.createTimer(
    ros::Duration(1.0 / getParamOrThrow<double>(t_nh, "brick_pickup/rate")), 
    &uav_sm::BrickPickupStateMachine::update_state, this);
  m_initFilterTimer = t_nh.createTimer(
    ros::Duration(1.0 / getParamOrThrow<double>(t_nh, "brick_pickup/color_init_rate")),
    &uav_sm::BrickPickupStateMachine::init_filter, this);
  m_publishTrajectoryTimer = t_nh.createTimer(
    ros::Duration(1.0 / getParamOrThrow<double>(t_nh, "brick_pickup/search_traj_rate")),
    &uav_sm::BrickPickupStateMachine::publish_trajectory, this);
}

private:

inline const VisualServoState getCurrentVisualServoState() {
  return static_cast<VisualServoState>(m_handlerVSSMState.getData().data);
}

void update_state(const ros::TimerEvent& /* unused */) {
  
  if (m_currentStatus.isApproaching() && is_close_to_brick()) {
    ROS_WARN("BrickPickup::update_state - SEARCH state activated");
    m_currentStatus.m_status = BrickPickupStates::SEARCH;
    m_uavTrajectory.clear();
    return;
  }

  if (m_currentStatus.isSearching() 
       && getCurrentVisualServoState() == VisualServoState::OFF
       && toggle_visual_servo_state_machine(true)) {
    ROS_WARN("BrickPickup::update_state - ATTEMPT_PICKUP activated");
    m_currentStatus.m_status = BrickPickupStates::ATTEMPT_PICKUP;
    return;
  }

  if (m_currentStatus.isAttemptingPickup() 
      && getCurrentVisualServoState() == VisualServoState::OFF) {
    ROS_WARN("BrickPickup::update_state - VisualServoState is OFF!.");

    m_uavTrajectory.clear(); 
    if (is_brick_picked_up()) {
      ROS_INFO("BrickPickup::update_state - brick is picked up, DROPOFF state activated.");
      m_currentStatus.m_status = BrickPickupStates::DROPOFF;
      
    } else {
      ROS_FATAL("BrickPickup::update_state - brick is not picked up, APPROACH state activated");
      m_currentStatus.m_status = BrickPickupStates::APPROACH;
    }
    return;
  }

  if (m_currentStatus.isDropOff() 
      && !is_brick_picked_up()) {
    ROS_WARN("BrickPickup::update_state - DROPOFF finished, SEARCH state activated");
    m_currentStatus.m_status = BrickPickupStates::SEARCH;
    m_uavTrajectory.clear();
  }
}

void init_filter(const ros::TimerEvent& /* unused */) {
  if (m_currentStatus.isSearching()) {
    ROS_INFO("BrickPickup::init_filter - initializing color %s", 
      m_currentStatus.m_brickColor.c_str());
    filter_choose_color(m_currentStatus.m_brickColor);
  }
}

void publish_trajectory(const ros::TimerEvent& /* unused */) {
  
  if (m_currentStatus.isSearching()) {    
    if ( m_uavTrajectory.empty()) {
      ROS_WARN("publish_trajectory - generating search trajectory.");
      m_uavTrajectory = uav_reference::traj_gen::generateCircleTrajectoryAroundPoint(
        m_currentStatus.m_localBrick.x(), m_currentStatus.m_localBrick.y(), 
        m_currentStatus.m_localBrick.z(), 200);
    }

    m_pubPositionHold.publish(m_uavTrajectory.front());
    m_uavTrajectory.pop_front();
   }

  if (m_currentStatus.isApproaching() && !is_close_to_brick()) {
    if (m_uavTrajectory.empty()) {
      ROS_WARN("publish_trajectory - generating approach trajectory.");
      m_uavTrajectory = uav_reference::traj_gen::generateLinearTrajctory(
        m_currentStatus.m_localBrick.x(), m_currentStatus.m_localBrick.y(), 
        m_currentStatus.m_localBrick.z(), m_handlerOdometry.getData(), 500); 
    }
    
    m_pubPositionHold.publish(m_uavTrajectory.front());
    m_uavTrajectory.pop_front();
  }
} 

bool brick_pickup_global_cb(GeoBrickReq& request, GeoBrickResp& response) {
  // If we want to disable the global brick pickup
  if (!request.enable) {
    ROS_FATAL("BPSM::brick_pickup_global_cb - brick_pickup/global disabled");
    response.status = false;
    m_currentStatus.m_status = BrickPickupStates::OFF;
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

  m_uavTrajectory.clear();
  return true;
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
  ROS_INFO_COND(resp.success, "BrickPickup - VSSM activated");
  return resp.success;
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

bool is_close_to_brick() {
  return uav_reference::traj_gen::isCloseToReference(
    uav_reference::traj_gen::toTrajectoryPointMsg(
      m_currentStatus.m_localBrick.x(), m_currentStatus.m_localBrick.y(), 
      m_currentStatus.m_localBrick.z()),
    m_handlerOdometry.getData(), 2);
}

bool is_brick_picked_up() {
  return m_handlerBrickAttached.getData().data ==  true;
}

std::list<trajectory_msgs::MultiDOFJointTrajectoryPoint> m_uavTrajectory;
ros::Publisher m_pubPositionHold;
ros::Timer m_runTimer, m_initFilterTimer, m_publishTrajectoryTimer;
Global2Local m_global2Local;
BrickPickupStatus m_currentStatus;
ros::ServiceServer m_serviceBrickPickup;
ros::ServiceClient m_vssmCaller, m_chooseColorCaller;
TopicHandler<std_msgs::Int32> m_handlerVSSMState;
TopicHandler<nav_msgs::Odometry> m_handlerOdometry;
TopicHandler<std_msgs::Bool> m_handlerBrickAttached;
std::unique_ptr<ParamHandler<PickupParams>> m_pickupConfig;
};
}
#endif
