#ifndef VISUAL_SERVO_STATE_MACHINE_H
#define VISUAL_SERVO_STATE_MACHINE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>

namespace uav_reference 
{
    
enum VisualServoState {
    OFF,
    BRICK_ALIGNMENT,
    DESCENT,
    TOUCHDOWN
};

class VisualServoStateMachine
{

public:

VisualServoStateMachine(ros::NodeHandle& nh)
{
    // Define Publishers
    _pubVisualServoPoseSp = 
        nh.advertise<geometry_msgs::PoseStamped>("visual_servo/sm/pose", 1);

    // Define Subscribers
    _subOdom =
        nh.subscribe("odometry", 1, &uav_reference::VisualServoStateMachine::odomCb, this);
    _subTargetError =  
        nh.subscribe("visual_servo/sm/error", 1, &uav_reference::VisualServoStateMachine::errorCb, this);
    
}

~VisualServoStateMachine()
{

}

void odomCb(const nav_msgs::OdometryConstPtr& msg)
{

}

void errorCb(const std_msgs::Float64ConstPtr& msg)
{

}
    
private:

    VisualServoState _currentState = VisualServoState::OFF;
    ros::Publisher _pubVisualServoPoseSp;
    ros::Subscriber _subOdom;
    ros::Subscriber _subTargetError;
};
}

#endif /* VISUAL_SERVO_STATE_MACHINE_H */