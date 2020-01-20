#include <std_msgs/Int32.h>
#include <ros/ros.h>
#include <uav_ros_control/BrickPickupStateMachineParametersConfig.h>
#include <uav_ros_control_msgs/GeoBrickApproach.h>
#include <uav_ros_control/filters/Util.h>

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
  BrickPickupStatus() : BrickPickupStatus("default", 0, 0, 0) { }
  BrickPickupStatus(
    const std::string& t_brickColor, const double t_globalLat, const double t_globalLon,
    const double t_globalAltRel, const BrickPickupStates t_status = BrickPickupStates::OFF) :
      m_brickColor(t_brickColor),
      m_globalLat(t_globalLat),
      m_globalLon(t_globalLon),
      m_globalAltRel(t_globalAltRel),
      m_status(t_status) { }

  BrickPickupStates m_status;
  std::string m_brickColor;
  double m_globalLat, m_globalLon, m_globalAltRel;
};

typedef uav_ros_control::BrickPickupStateMachineParametersConfig PickupParams;
typedef uav_ros_control_msgs::GeoBrickApproach GeoBrickMsg;
typedef GeoBrickMsg::Request GeoBrickReq;
typedef GeoBrickMsg::Response GeoBrickResp;

class BrickPickupStateMachine {

public:
BrickPickupStateMachine(ros::NodeHandle& t_nh) :
    m_handlerVSSMState(t_nh, "visual_servo_sm/state") {
  initializeParameters(t_nh);
  
  m_serviceBrickPickup = t_nh.advertiseService(
    "brick_pickup/global",
    &uav_sm::BrickPickupStateMachine::brick_pickup_global_cb,
    this);
  
  m_runTimer = t_nh.createTimer(
    ros::Duration(1.0 / getParamOrThrow<double>(t_nh, "brick_pickup/rate")), 
    &uav_sm::BrickPickupStateMachine::run_once, this);
}

private:
bool brick_pickup_global_cb(GeoBrickReq& request, GeoBrickResp& response) {
  
  // If we want to disable the global brick pickup
  if (!request.enable) {
    ROS_FATAL("BPSM::brick_pickup_global_cb - brick_pickup/global disabled");
    response.status = false;
    //m_currentState == BrickPickupStates::OFF;
    return true;
  }

  // Enable global brick pickup
  m_currentStatus = BrickPickupStatus(
    request.brick_color, request.latitude, request.longitude, 
    request.altitude_relative, BrickPickupStates::APPROACH);
}

void run_once(const ros::TimerEvent& /* unused */) {
  
}

inline const VisualServoState getCurrentVisualServoState() {
  return static_cast<VisualServoState>(m_handlerVSSMState.getData().data);
}

void initializeParameters(ros::NodeHandle& nh) {
  PickupParams initParams;
  initParams.dummy_param = getParamOrThrow<double>(nh, "brick_pickup/dummy_param");
  m_pickupConfig.reset(new ParamHandler<PickupParams>(initParams, "brick_pickup"));
}

BrickPickupStatus m_currentStatus;
ros::ServiceServer m_serviceBrickPickup;
ros::Timer m_runTimer;
TopicHandler<std_msgs::Int32> m_handlerVSSMState;
std::unique_ptr<ParamHandler<PickupParams>> m_pickupConfig;
};
}
