//
// Created by robert on 20. 09. 2019..
//

#ifndef UAV_ROS_CONTROL_VISUALSERVO_H
#define UAV_ROS_CONTROL_VISUALSERVO_H

#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf/transform_listener.h>


#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/service_server.h>


namespace uav_reference {
 /**
  * Publish UAV reference based on the error reported by the drone's color filter.
  * The reference is published in the global coordinate system.
  **/
  class VisualServo {
      public:
        /**
         * Default constructor.
         */
        VisualServo(ros::NodeHandle&);
        virtual ~VisualServo();

        /**
         * Update position setpoint.
         */
         void updateSetpoint();

         /**
          * Publish new setpoint as MultiDOFJointTrajectoryPoint
          */
          void publishNewSetpoint();

          // fali updateCarrotStatus();

          bool isVisualServoEnabled();

   private:

      void intializeParameters();

      /**
       * Callback function for StartVisualServo service
       */
       bool startVisualServoServiceCb(std_srvs::Empty::Request& request,
           std_srvs::Empty::Response& response);

        /**
         * Odometry callback function for extracting the UAV's pose.
         */
         void odomCb(const nav_msgs::OdometryConstPtr&);

         void pitchErrorCb(const std_msgs::Float32);
         void yawErrorCb(const std_msgs::Float32);

         std::array<double, 3> _uavPos{0.0, 0.0, 0.0};
         std::array<double, 3> _setpointPosition{0.0, 0.0, 0.0};
         double _dx, _dy, _uav_yaw;

         //bool _positionHold = false;
         bool _visualServoEnabled = false;

         /** Publishers */
         ros::Publisher _pubNewSetpoint;

         /** Subscribers */
         ros::Subscriber _subOdom;
         ros::Subscriber _subYawError, _subPitchError; // As published by erl_husky color_filter node.

         /** Services */
         ros::ServiceServer _serviceStartVisualServo;

         /** TF **/
         tf::TransformListener tf_listener;

  };
}



#endif //UAV_ROS_CONTROL_VISUALSERVO_H
