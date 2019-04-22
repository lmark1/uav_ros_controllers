#ifndef PID_H
#define PID_H

#include <uav_ros_control_msgs/PIDController.h>
class PID
{
	private:
		float kp, ki, kd, up, ui, ui_old, ud, u;
		float lim_high, lim_low, ref, meas, error_old;

		bool firstPass;
		
	public:
		PID();
		void resetPIDParams();
		void resetIntegrator();
		void set_kp(float invar);
		float get_kp();
		float& get_kp_ref();
		void set_ki(float invar);
		float get_ki();
		float& get_ki_ref();
		void set_kd(float invar);
		float get_kd();
		float& get_kd_ref();
		void set_lim_high(float invar);
		float get_lim_high();
		float& get_lim_high_ref();
		void set_lim_low(float invar);
		float get_lim_low();
		float& get_lim_low_ref();
		float compute(float ref_, float meas_, float dt_);
		void get_pid_values(float *up_, float *ui_, float *ud_, float *u_);
		void create_msg(uav_ros_control_msgs::PIDController &msg);

		friend std::ostream& operator << (std::ostream&, const PID&);
};

#endif