#ifndef PICKUP_STATE_MACHINE_H
#define PICKUP_STATE_MACHINE_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <std_srvs/SetBool.h>

#include <uav_ros_control/filters/Util.h>
#include <uav_ros_control/VisualServoStateMachineParametersConfig.h>

#include <string>
#include <unordered_map>
#include <iostream>

namespace uav_reference 
{

typedef trajectory_msgs::MultiDOFJointTrajectoryPoint TrajPoint;
typedef uav_ros_control::VisualServoStateMachineParametersConfig PickupParams;

enum class PickupState 
{
  OFF,
  ALIGNMENT,
  DESCENT,
  PICKUP
};

std::ostream& operator<<(std::ostream& o, const PickupState& state) 
{
  switch (state) {
    case PickupState::OFF:
      o << "[OFF]";
      break;
    
    case PickupState::ALIGNMENT:
      o << "[ALIGNMENT]";
      break;

    case PickupState::DESCENT:
      o << "[DESCENT]";
      break;
    
    case PickupState::PICKUP:
      o << "[PICKUP]";
      break;
  }
  return o;
}

class PickupStateMachine 
{
public:

explicit PickupStateMachine(ros::NodeHandle& t_nh) : 
  m_handlerYawError(t_nh, "yaw_error"),
  m_handlerGlobalCentroid(t_nh, "centroid_global"),
  m_handlerLocalCentroid(t_nh, "centroid_local"),
  m_handlerOdometry(t_nh, "odometry"),
  m_currentState(PickupState::OFF)
{
  m_pubTrajectoryPoint = t_nh.advertise<TrajPoint>("trajectory_point", 1);
  // Setup brick pickup service callback
  m_serviceBrickPickup = t_nh.advertiseService(
    "brick_pickup",
    &uav_reference::PickupStateMachine::brickPickupServiceCb,
    this);
    
  initializeParameters(t_nh);
  initializeStateActions();
  initializeStateTransitions();
}

private:

bool brickPickupServiceCb(
  std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
  if (m_currentState != PickupState::OFF && request.data) {
    ROS_WARN("PickupStateMachine: - pickup is already active");
    response.success = true;
    response.message = "Pickup is already active";
    return true;
  }

  if (m_currentState == PickupState::OFF && request.data) {
    ROS_WARN("PickupStateMachine - attempting to start pickup");
    bool success = m_stateTransitionMap[m_currentState](PickupState::ALIGNMENT);
    response.success = success;
    response.message = "Pickup attempt - check success";
  }
}

void initializeStateTransitions()
{
  const auto offTransition = [this] (const PickupState& state) {
    return false;
  };

  const auto descentTransition = [this] (const PickupState& state) {
    return false;
  };

  const auto alignmentTransition = [this] (const PickupState& state) {
    return false;
  };

  const auto pickupTransition = [this] (const PickupState& state) {
    return false;
  };

  m_stateTransitionMap.emplace(PickupState::OFF, offTransition);
  m_stateTransitionMap.emplace(PickupState::DESCENT, descentTransition);
  m_stateTransitionMap.emplace(PickupState::ALIGNMENT, alignmentTransition);
  m_stateTransitionMap.emplace(PickupState::PICKUP, pickupTransition);
}

void initializeStateActions() 
{
  const auto offStateAction = [this] () {
    // Do nothing
  };

  const auto descentStateAction = [this] () {
    // TODO: ...
  };

  const auto alignmentStateAction = [this] () {
    // TODO: ...
  };

  const auto pickupStateAction = [this] () {
    // TODO: ...
  };
  
  m_stateActionMap.emplace(PickupState::OFF, offStateAction);
  m_stateActionMap.emplace(PickupState::ALIGNMENT, alignmentStateAction);
  m_stateActionMap.emplace(PickupState::DESCENT, descentStateAction);
  m_stateActionMap.emplace(PickupState::PICKUP, pickupStateAction);
}

void initializeParameters(ros::NodeHandle& t_nh) 
{
  PickupParams paramConfig;
  paramConfig.after_touchdown_height = ros_util::getParamOrThrow<double>(
    t_nh, "visual_servo/state_machine/after_touchdown_height");
  paramConfig.ascent_speed = ros_util::getParamOrThrow<double>(
    t_nh, "visual_servo/state_machine/ascent_speed");
  paramConfig.brick_alignment_height = ros_util::getParamOrThrow<double>(
    t_nh, "visual_servo/state_machine/brick_alignment_height");
  paramConfig.descent_speed = ros_util::getParamOrThrow<double>(
    t_nh, "visual_servo/state_machine/descent_speed");
  paramConfig.detection_counter = ros_util::getParamOrThrow<double>(
    t_nh, "visual_servo/state_machine/detection_counter");
  paramConfig.disable_visual_servo_touchdown_height = ros_util::getParamOrThrow<double>(
    t_nh, "visual_servo/state_machine/disable_visual_servo_touchdown_height");
  paramConfig.magnet_offset = ros_util::getParamOrThrow<double>(
    t_nh, "visual_servo/state_machine/disable_visual_servo_touchdown_height");
  paramConfig.min_error = ros_util::getParamOrThrow<double>(
    t_nh, "visual_servo/state_machine/min_error");
  paramConfig.min_touchdown_align_duration = ros_util::getParamOrThrow<double>(
    t_nh, "visual_servo/state_machine/min_touchdown_align_duration");
  paramConfig.min_touchdown_target_position_error_xy = ros_util::getParamOrThrow<double>(
    t_nh, "visual_servo/state_machine/min_touchdown_target_position_error_xy");
  paramConfig.min_touchdown_target_position_error_z = ros_util::getParamOrThrow<double>(
    t_nh, "visual_servo/state_machine/min_touchdown_target_position_error_z");
  paramConfig.min_touchdown_uav_velocity_error_xy = ros_util::getParamOrThrow<double>(
    t_nh, "visual_servo/state_machine/min_touchdown_uav_velocity_error_xy");
  paramConfig.min_touchdown_uav_velocity_error_z = ros_util::getParamOrThrow<double>(
    t_nh, "visual_servo/state_machine/min_touchdown_uav_velocity_error_z");
  paramConfig.min_yaw_error = ros_util::getParamOrThrow<double>(
    t_nh, "visual_servo/state_machine/min_yaw_error");
  paramConfig.touchdown_height = ros_util::getParamOrThrow<double>(
    t_nh, "visual_servo/state_machine/touchdown_height");
  paramConfig.touchdown_speed = ros_util::getParamOrThrow<double>(
    t_nh, "visual_servo/state_machine/touchdown_speed");
  
  m_handlerParams.reset(
    new ros_util::ParamHandler<PickupParams>(paramConfig, "pickup_config"));
}

static constexpr int INVALID_DISTANCE = -1;

ros::Publisher m_pubTrajectoryPoint;
TrajPoint m_currentTrajectoryPoint;

ros_util::TopicHandler<std_msgs::Float32> m_handlerYawError;
ros_util::TopicHandler<geometry_msgs::Vector3> m_handlerGlobalCentroid, m_handlerLocalCentroid;
ros_util::TopicHandler<nav_msgs::Odometry> m_handlerOdometry;
std::unique_ptr<ros_util::ParamHandler<PickupParams>> m_handlerParams;

PickupState m_currentState;
std::unordered_map<PickupState, std::function<void()>, ros_util::EnumClassHash> m_stateActionMap;
std::unordered_map<PickupState, std::function<bool(const PickupState&)>, ros_util::EnumClassHash> m_stateTransitionMap;
ros::ServiceServer m_serviceBrickPickup;
};

}

#endif // PICKUP_STATE_MACHINE