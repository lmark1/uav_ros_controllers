#ifndef MASTER_PICKUP_CONTROL_H
#define MASTER_PICKUP_CONTROL_H

#include <ros/ros.h>
#include <uav_ros_control/filters/Util.h>
#include <uav_ros_control/reference/PickupStates.h>
#include <uav_ros_control/reference/TrajectoryGenerator.h>

#include <mavros_msgs/State.h>
#include <std_msgs/String.h>
#include <mavros_msgs/CommandBool.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <uav_search/GetPoints.h>

#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <uav_ros_control_msgs/TakeOff.h>

using namespace ros_util;
using namespace pickup_states;
using namespace uav_reference;

namespace uav_sm 
{

class MasterPickupControl
{
public:

MasterPickupControl(ros::NodeHandle& t_nh) :
  m_handlerState(t_nh, "mavros/state"),
  m_handlerGpsFix(t_nh, "mavros/global_position/global"),
  m_handlerCarrotStatus(t_nh, "carrot/status"),
  m_handlerOdometry(t_nh, "mavros/global_position/local"),
  m_currentState(MasterPickupStates::OFF)
{
  // Initialize publisher 
  m_pubTrajGen = t_nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
    "topp/input/trajectory", 1
  );

  // Advertise service
  m_serviceMasterPickup = t_nh.advertiseService(
    "brick_pickup/master",
    &uav_sm::MasterPickupControl::master_pickup_cb,
    this
  );

  m_clientArming = t_nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  m_clientTakeoff = t_nh.serviceClient<uav_ros_control_msgs::TakeOff>("takeoff");
  m_clientLand = t_nh.serviceClient<std_srvs::SetBool>("land");
  m_clientSearchGenerator = t_nh.serviceClient<uav_search::GetPoints>("get_points");
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

  if (!arm_uav()) {
    ROS_FATAL("MasterPickupControl - request denied, ARMING failed.");
    set_response(false);
    return true;
  }

  sleep_for(ARM_DURATION);
  
  // Assume vehicle is armed at this point
  if (!takeoff_uav()) {
    ROS_FATAL("MasterPickupControl - request denied, TAKEOFF unsuccessful");
    set_response(false);
    return true;
  }
  sleep_for(TAKEOFF_DURATION);

  // Assume takeoff is successful at this point
  ROS_INFO("MasterPickupControl - request approved, TAKEOFF successful.");
  switch_to_search_state();
  set_response(true);
  return true;
}

void switch_to_off_state()
{
  ROS_WARN_STREAM(m_currentState << " -> " << MasterPickupStates::OFF);
  clear_current_trajectory();
  go_to_home();
  land_uav();
  m_currentState = MasterPickupStates::OFF;
}

void switch_to_search_state()
{
  ROS_WARN_STREAM(m_currentState << " -> " << MasterPickupStates::SEARCH);
  clear_current_trajectory();
  generate_search_trajectory();
  m_currentState = MasterPickupStates::SEARCH;
}

void generate_search_trajectory()
{
  uav_search::GetPoints::Request pointsRequest;
  uav_search::GetPoints::Response pointsResponse;
  pointsRequest.request.height = SEARCH_HEIGHT;
  pointsRequest.request.pattern = "lawn";
  pointsRequest.request.size_x = 20;
  pointsRequest.request.size_y = 20;
  pointsRequest.request.spacing = 2;

  if (!m_clientSearchGenerator.call(pointsRequest, pointsResponse)) {
    ROS_FATAL("MasterPickupcontrol::generate_search_trajectory - unable to call TRAJECTORY generator.");
    return;
  }

  const auto valid_trajectory = [&pointsResponse] () { 
    return pointsResponse.response.size_x * pointsResponse.response.size_y 
      == pointsResponse.response.data.size();
  };
  if (!valid_trajectory()) {
    ROS_FATAL("MasterPickupControl::generate_search_trajectory - invalid trajectory recieved.");
    return;
  }

  trajectory_msgs::MultiDOFJointTrajectory searchtrajectory;
  searchtrajectory.header.stamp = ros::Time::now();
  searchtrajectory.points.push_back(
    traj_gen::toTrajectoryPointMsg(
      m_handlerOdometry.getData().pose.pose.position.x,
      m_handlerOdometry.getData().pose.pose.position.y,
      m_handlerOdometry.getData().pose.pose.position.z,
      m_handlerOdometry.getData().pose.pose.orientation.x,
      m_handlerOdometry.getData().pose.pose.orientation.y,
      m_handlerOdometry.getData().pose.pose.orientation.z,
      m_handlerOdometry.getData().pose.pose.orientation.w
    )
  );

  for (std::size_t i = 0; 
      i < pointsResponse.response.data.size(); i+= pointsResponse.response.size_y) {
    const double newX = pointsResponse.response.data[i];
    const double newY = pointsResponse.response.data[i+1];

    tf2::Quaternion q = traj_gen::getHeadingQuaternion(
      searchtrajectory.points.back().transforms[0].translation.x,
      searchtrajectory.points.back().transforms[0].translation.y,
      newX, newY
    );

    searchtrajectory.points.push_back(
      traj_gen::toTrajectoryPointMsg(
        newX, newY, SEARCH_HEIGHT,
        q.getX(), q.getY(), q.getZ(), q.getW()
      )
    ); 
  } // end for
  clear_current_trajectory();
  m_pubTrajGen.publish(searchtrajectory);
}

void clear_current_trajectory() 
{
  ROS_WARN("clear_current_trajectory - Clearing trajectory");
  m_pubTrajGen.publish(trajectory_msgs::MultiDOFJointTrajectory());
  ros::Duration(1.0).sleep();
}

void go_to_home() 
{
  ROS_INFO("MasterPickupControl::go_to_home");
  double homeAltitude = m_handlerOdometry.getData().pose.pose.position.z; 
  m_pubTrajGen.publish(traj_gen::generateLinearTrajectory_topp(
    0, 0, homeAltitude,
    m_handlerOdometry.getData()
    )
  );

  const auto is_close_to_home = [this, &homeAltitude] () {
    return traj_gen::isCloseToReference(
      traj_gen::toTrajectoryPointMsg(0, 0, homeAltitude, 0), 
      this->m_handlerOdometry.getData(), GOTO_HOME_TOL);
  };

  while (!is_close_to_home()) {
    ROS_INFO("MasterPickupControl::go_to_home - going home ...");
    ros::Duration(1.0).sleep();
  }

  // Arrived home
  clear_current_trajectory();
}

bool arm_uav()
{
  ROS_WARN("MasterPickupControl - arming UAV.");
  mavros_msgs::CommandBool::Request armRequest;
  mavros_msgs::CommandBool::Response armResponse;
  armRequest.value = true;
  if (!m_clientArming.call(armRequest, armResponse)) {
    ROS_FATAL("MasterPickupControl::arm_uav - call to ARM service failed");
    return false;
  }

  if (!armResponse.success) {
    ROS_FATAL("MasterPickupControl::arm_uav - UAV arm failed");
    return false;
  }

  ROS_INFO("MasterPickupControl::arm_uav - UAV ARM successful");
  return true;
}

bool takeoff_uav()
{
  ROS_WARN("MasterPickupControl - UAV takeoff to %.2f", TAKEOFF_HEIGHT);
  uav_ros_control_msgs::TakeOff::Request takeoffRequest;
  uav_ros_control_msgs::TakeOff::Response takeoffResponse;
  takeoffRequest.rel_alt = TAKEOFF_HEIGHT;
  if (!m_clientTakeoff.call(takeoffRequest, takeoffResponse)) {
    ROS_FATAL("MasterPickupControl::takeoff_uav - call to TAKEOFF service failed.");
    return false;
  }

  if (!takeoffResponse.success) {
    ROS_FATAL("MasterPickupControl::takeoff_uav - TAKEOFF failed.");
    return false;
  }
  ROS_INFO("MasterPickupControl::takeoff_uav - TAKEOFF successful");
  return true;
}

void land_uav()
{
  ROS_WARN("MasterPickupControl - UAV land");
  std_srvs::SetBool::Request landRequest;
  std_srvs::SetBool::Response landResponse;
  landRequest.data = true;
  if (!m_clientLand.call(landRequest, landResponse)) {
    ROS_FATAL("MasterPickupControl::land_uav - call to LAND service failed");
    return;
  }

  if (!landResponse.success) {
    ROS_FATAL("MasterPickupControl::land_uav - LAND mode still not active.");
    return;
  }
  ROS_INFO("MasterPickupControl::laun_uav - UAV is landing."); 
}

static constexpr double ARM_DURATION = 3.0;
static constexpr double TAKEOFF_DURATION = 5.0;
static constexpr double TAKEOFF_HEIGHT = 3.0;
static constexpr double SEARCH_HEIGHT = 5.0;
static constexpr double GOTO_HOME_TOL = 1.0;

MasterPickupStates m_currentState;
  
ros::ServiceServer m_serviceMasterPickup;
ros::ServiceClient m_clientArming, m_clientTakeoff, m_clientLand, m_clientSearchGenerator;

ros::Publisher m_pubTrajGen;
TopicHandler<mavros_msgs::State> m_handlerState;
TopicHandler<sensor_msgs::NavSatFix> m_handlerGpsFix;
TopicHandler<std_msgs::String> m_handlerCarrotStatus;
TopicHandler<nav_msgs::Odometry> m_handlerOdometry;

};

}

#endif /* MASTER_PICKUP_CONTROL_H */