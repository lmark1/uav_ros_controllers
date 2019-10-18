//
// Created by robert on 20. 09. 2019..
//

#ifndef UAV_ROS_CONTROL_VISUALSERVO_H
#define UAV_ROS_CONTROL_VISUALSERVO_H

#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
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

#include <dynamic_reconfigure/server.h>
#include <uav_ros_control/VisualServoParametersConfig.h>
#include <uav_ros_control/control/PID.h>
#include <uav_ros_control/filters/NonlinearFilters.h>

namespace uav_reference {
/**
 * Name of dynamic reconfigure node.
 */
#define VISUAL_SERVO_DYN_RECONF "visual_servo_config"

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

     void setRate(double new_rate) {_rate =  new_rate;}

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
      void nContoursCb(const std_msgs::Int32&);
      void clickerCb(const std_msgs::Bool&);

      // X and Y axes of the image coordinate frame.
      PID _x_axis_PID, _y_axis_PID;

      PID _yaw_PID;

      // TODO: To be used for the UAV pursuit scenario
      PID _distance_PID, _z_axis_PID;

      int _n_contours;
      std::array<double, 3> _uavPos{0.0, 0.0, 0.0};
      std::array<double, 3> _setpointPosition{0.0, 0.0, 0.0};
      double _error_x, _error_y, _offset_x, _offset_y, _deadzone_x, _deadzone_y;  // brick laying scenario
      double _error_z, _error_yaw, _error_distance, _offset_distance; // drone pursuit scenario
      double _uavYaw, _setpointYaw;
      double _coordinate_frame_yaw_difference;
      double _deadzone_yaw;
      double _rate;

      nav_msgs::Odometry _current_odom;

      // The x and y offset needed to align the magnetic gripper with the magnetic patch at z = 1m and z = 2m.
      double _offset_x_1, _offset_x_2, _offset_y_1, _offset_y_2; // Determine these experimentally.
      double _visual_servo_shutdown_height;
      double _landing_speed, _landing_range_x, _landing_range_y, _landing_range_yaw;

      double _qx, _qy, _qz, _qw;

      //bool _positionHold = false;
      bool _visualServoEnabled = false;
      bool _use_imu = false;
      bool _use_odometry;
      bool _brick_laying_scenario, _pickup_allowed;
      bool _clicker_clicked;

      /** Publishers */
      ros::Publisher _pubNewSetpoint;
      trajectory_msgs::MultiDOFJointTrajectoryPoint _new_point;

      /** Subscribers */
      ros::Subscriber _subOdom, _subImu;
      ros::Subscriber _subXError, _subYError, _subZError, _subYawError, _subPitchError, _subNContours;

      /** Services */
      ros::ServiceServer _serviceStartVisualServo;

      void visualServoParamsCb(
          uav_ros_control::VisualServoParametersConfig& configMsg, uint32_t level);

      /** Define Dynamic Reconfigure parameters **/
      boost::recursive_mutex _VSConfigMutex;
      dynamic_reconfigure::
      Server<uav_ros_control::VisualServoParametersConfig>
          _VSConfigServer {_VSConfigMutex, ros::NodeHandle(VISUAL_SERVO_DYN_RECONF)};
      dynamic_reconfigure::
      Server<uav_ros_control::VisualServoParametersConfig>::CallbackType
          _VSParamCallback;
  };

  void runDefault(VisualServo& cc, ros::NodeHandle& nh);
}

#endif //UAV_ROS_CONTROL_VISUALSERVO_H
