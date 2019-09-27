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
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf2/LinearMath/Quaternion.h>

#include <ros/ros.h>
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

     bool isVisualServoEnabled();

   private:

      /**
       * Callback function for StartVisualServo service
       */
      bool startVisualServoServiceCb(std_srvs::Empty::Request& request,
          std_srvs::Empty::Response& response);

      /**
        * Odometry callback function for extracting the UAV's pose.
        */
      void odomCb(const nav_msgs::OdometryConstPtr&);
      void imuCb (const sensor_msgs::ImuConstPtr&);

      /**
       * Callback functions for the visual servo process values.
       *
       * Use xErrorCb and yErrorCb when the camera is facing the floor (brick laying scenario).
       * When the camera is facing forward please use the yaw and pitch callbacks (drone pursuit scenario).
       */
      void xErrorCb(const std_msgs::Float32&);
      void yErrorCb(const std_msgs::Float32&);
      void zErrorCb(const std_msgs::Float32&);
      void yawErrorCb(const std_msgs::Float32&);
      void pitchErrorCb(const std_msgs::Float32&);

      std::array<double, 3> _uavPos{0.0, 0.0, 0.0};
      std::array<double, 3> _setpointPosition{0.0, 0.0, 0.0};
      double _dx, _dy, _offset_x, _offset_y, _deadzone_x, _deadzone_y;  // brick laying scenario
      double _dz, _dYaw, _dDistance; // drone pursuit scenario
      double _uavYaw, _setpointYaw;
      double _gain_dx, _gain_dy, _gain_dz, _gain_dYaw, _gain_dDistance;
      double _move_saturation;
      double _coordinate_frame_yaw_difference;
      double _yaw_error_integrator, _yaw_error_integrator_gain;
      double _yaw_error_integrator_clamp, _yaw_error_integrator_deadzone;

      double _qx, _qy, _qz, _qw;

      //bool _positionHold = false;
      bool _visualServoEnabled = false;
      bool _use_imu = false;

      /** Publishers */
      ros::Publisher _pubNewSetpoint;
      trajectory_msgs::MultiDOFJointTrajectoryPoint _new_point;

      /** Subscribers */
      ros::Subscriber _subOdom, _subImu;
      ros::Subscriber _subXError, _subYError, _subZError, _subYawError, _subPitchError;

      /** Services */
      ros::ServiceServer _serviceStartVisualServo;

      void getParameters();
  };

  void runDefault(VisualServo& cc, ros::NodeHandle& nh);
}

#endif //UAV_ROS_CONTROL_VISUALSERVO_H
