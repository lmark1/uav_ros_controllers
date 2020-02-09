#ifndef MASTER_PICKUP_CONTROL_H
#define MASTER_PICKUP_CONTROL_H

#include <ros/ros.h>
#include <uav_ros_control/filters/Util.h>
#include <uav_ros_control/reference/PickupStates.h>

#include <mavros_msgs/State.h>
#include <std_msgs/String.h>
#include <mavros_msgs/CommandBool.h>
#include <sensor_msgs/NavSatFix.h>

#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <uav_ros_control_msgs/TakeOff.h>

using namespace ros_util;
using namespace pickup_states;
namespace uav_sm 
{

class MasterPickupControl
{
public:

MasterPickupControl(ros::NodeHandle& t_nh) :
  m_handlerState(t_nh, "mavros/state"),
  m_handlerGpsFix(t_nh, "mavros/global_position/global"),
  m_handlerCarrotStatus(t_nh, "carrot/status"),
  m_currentState(MasterPickupStates::OFF)
{
  // Advertise service
  m_serviceMasterPickup = t_nh.advertiseService(
    "brick_pickup/master",
    &uav_sm::MasterPickupControl::master_pickup_cb,
    this
  );

  m_clientArming = t_nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  m_clientTakeoff = t_nh.serviceClient<uav_ros_control_msgs::TakeOff>("takeoff");
  m_clientLand = t_nh.serviceClient<std_srvs::Empty>("land");
}

private:

inline static void sleep_for(double duration) { ros::Duration(duration).sleep(); }
inline bool is_guided_active() { return m_handlerState.getData().mode == "GUIDED_NOGPS"; }
inline bool is_land_active() { return m_handlerState.getData().mode == "LAND"; }
inline bool is_vehicle_armed() { return m_handlerState.getData().armed; }
inline bool in_off_state() const { return m_currentState == MasterPickupStates::OFF; }
inline bool in_search_state() const { return m_currentState == MasterPickupStates::SEARCH; }
inline bool in_task_state() const { return m_currentState == MasterPickupStates::ACTION; }
inline bool is_uav_airborne() { 
  return m_handlerCarrotStatus.getData().data == "CARROT_ON_AIR" 
    && m_handlerCarrotStatus.getData().data == "HOLD";
}

bool master_pickup_cb(std_srvs::SetBool::Request& request, 
  std_srvs::SetBool::Response& response)
{
  const auto deactivation_requested = [&request] () { return !request.data; };
  const auto set_response = [&response] (bool success) { response.success = success; };

  if (deactivation_requested() && in_off_state()) {
    ROS_FATAL("MasterPickupControl - deactivation requested, already in OFF state.");
    set_response(false);
    return true;
  }

  if (deactivation_requested() && !in_off_state()) {
    ROS_INFO("MasterPickupControl - deactivation requested.");
    switch_to_off_state();
    set_response(false);
    return true;
  }
  
  // Assume mission is requested at this point
  if (!is_guided_active()) {
    ROS_FATAL("MasterPickupControl - request denied, not in GUIDED_NOGPS");
    set_response(false);
    return true;
  }

  if (is_vehicle_armed()) {
    ROS_FATAL("MasterPickupControl - request denied, vehicle is already ARMED");
    set_response(false);
    return true;
  }

  arm_uav();
  sleep_for(ARM_DURATION);
  if (!is_vehicle_armed()) {
    ROS_FATAL("MasterPickupControl - request denied, ARMING failed.");
    set_response(false);
    return true;
  }

  // Assume vehicle is armed at this point
  takeoff_uav();
  sleep_for(TAKEOFF_DURATION);
  if (!is_uav_airborne()) {
    ROS_FATAL("MasterPickupControl - request denied, TAKEOFF unsuccessful");
    set_response(false);
    return true;
  }

  ROS_INFO("MasterPickupControl - request approved, TAKEOFF successful.");
  switch_to_search_state();
  set_response(true);
  return true;
}

void switch_to_off_state()
{
  ROS_WARN_STREAM(m_currentState << " -> " << MasterPickupStates::OFF);
  land_uav();
  m_currentState = MasterPickupStates::OFF;
}

void switch_to_search_state()
{
  ROS_WARN_STREAM(m_currentState << " -> " << MasterPickupStates::SEARCH);
  // TODO: Generate a lawnmover search trajectory here
  // TODO: Handle switching to off state
  m_currentState = MasterPickupStates::SEARCH;
}

void arm_uav()
{
  ROS_WARN("MasterPickupControl - arming UAV.");
  mavros_msgs::CommandBool::Request armRequest;
  mavros_msgs::CommandBool::Response armResponse;
  armRequest.value = true;
  if (!m_clientArming.call(armRequest, armResponse)) {
    ROS_FATAL("MasterPickupControl::arm_uav - call to ARM service failed");
    return;
  }

  if (!armResponse.success) {
    ROS_FATAL("MasterPickupControl::arm_uav - UAV arm failed");
    return;
  }
  ROS_INFO("MasterPickupControl::arm_uav - UAV ARM successful");
}

void takeoff_uav()
{
  ROS_WARN("MasterPickupControl - UAV takeoff to %.2f", TAKEOFF_HEIGHT);
  uav_ros_control_msgs::TakeOff::Request takeoffRequest;
  uav_ros_control_msgs::TakeOff::Response takeoffResponse;
  takeoffRequest.rel_alt = TAKEOFF_HEIGHT;
  if (!m_clientTakeoff.call(takeoffRequest, takeoffResponse)) {
    ROS_FATAL("MasterPickupControl::takeoff_uav - call to TAKEOFF service failed.");
    return;
  }

  if (!takeoffResponse.success) {
    ROS_FATAL("MasterPickupControl::takeoff_uav - TAKEOFF failed.");
    return;
  }
  ROS_INFO("MasterPickupControl::takeoff_uav - TAKEOFF successful");
}

void land_uav()
{
  ROS_WARN("MasterPickupControl - UAV land");
  std_srvs::Empty::Request landRequest;
  std_srvs::Empty::Response landResponse;
  if (!m_clientLand.call(landRequest, landResponse)) {
    ROS_FATAL("MasterPickupControl::land_uav - call to LAND service failed");
    return;
  }

  if (!is_land_active()) {
    ROS_FATAL("MasterPickupControl::land_uav - LAND mode still not active.");
    return;
  } 
  ROS_INFO("MasterPickupControl::laun_uav - UAV is landing."); 
}

static constexpr double ARM_DURATION = 2.0;
static constexpr double TAKEOFF_DURATION = 2.0;
static constexpr double TAKEOFF_HEIGHT = 3.0;
MasterPickupStates m_currentState;
  
ros::ServiceServer m_serviceMasterPickup;
ros::ServiceClient m_clientArming, m_clientTakeoff, m_clientLand;
TopicHandler<mavros_msgs::State> m_handlerState;
TopicHandler<sensor_msgs::NavSatFix> m_handlerGpsFix;
TopicHandler<std_msgs::String> m_handlerCarrotStatus;
};

}

#endif /* MASTER_PICKUP_CONTROL_H */