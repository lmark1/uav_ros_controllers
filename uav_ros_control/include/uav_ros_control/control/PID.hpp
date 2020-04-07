#ifndef PID_H
#define PID_H

#include <uav_ros_control_msgs/PIDController.h>
#include <ros/ros.h>

#include <string>

class PID
{
private:
  float kp, ki, kd, up, ui, ui_old, ud, u;
  float lim_high, lim_low, ref, meas, error_old;

  bool firstPass;
  std::string _name;

public:
  PID();
  PID(std::string);
  void resetPIDParams();
  void resetIntegrator();
  void set_kp(float invar);
  float get_kp();
  void set_ki(float invar);
  float get_ki();
  void set_kd(float invar);
  float get_kd();
  void set_lim_high(float invar);
  float get_lim_high();
  void set_lim_low(float invar);
  float get_lim_low();
  float compute(float ref_, float meas_, float dt_);
  void get_pid_values(float *up_, float *ui_, float *ud_, float *u_);
  void create_msg(uav_ros_control_msgs::PIDController &msg);

  /**
   * Initialize PID parameters.
   *
   * @param nh - Node handle parameter
   * @param prefix - string prefix for parameter getting
   */
  void initializeParameters(ros::NodeHandle &nh, std::string prefix);
  friend std::ostream &operator<<(std::ostream &, const PID &);
};

#endif