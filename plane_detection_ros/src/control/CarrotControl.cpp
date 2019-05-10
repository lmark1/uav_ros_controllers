#include <plane_detection_ros/control/CarrotControl.h>
#include <uav_ros_control/PID.h>
#include <uav_ros_control/NonlinearFilters.h>

// Define all parameter paths here
#define PID_X_PARAM "/control/pos_y"
#define PID_Y_PARAM "/control/pos_y"
#define PID_Z_PARAM "/control/pos_z"
#define PID_VX_PARAM "/control/pos_y"
#define PID_VY_PARAM "/control/vel_y"
#define PID_VZ_PARAM "/control/vel_z"
#define HOVER_PARAM "/control/hover"
#define TOL_PARAM "/control/carrot_tol"

// Define deadzone constants
#define X_DEADZONE 0.01
#define Y_DEADZONE 0.01
#define Z_DEADZONE 0.01

carrot_control::CarrotControl::CarrotControl() :
    _posXPID {new PID ("Position - x")},
    _posYPID {new PID ("Position - y")}, 
    _posZPID {new PID ("Position - z")},
    _velXPID {new PID ("Velocity - x")},
    _velYPID {new PID ("Velocity - y")},
    _velZPID {new PID ("Velocity - z")}, 
	_hoverThrust (0),
    control_base::ControlBase(), 
	joy_control::JoyControl()
{
}

carrot_control::CarrotControl::~CarrotControl()
{
}

void carrot_control::CarrotControl::setCarrotPosition(double x, double y, double z)
{
	_carrotPos[0] = x;
	_carrotPos[1] = y;
	_carrotPos[2] = z;
}

void carrot_control::CarrotControl::updateCarrot()
{
	updateCarrotX(nonlinear_filters::deadzone(
		getXOffsetManual(), -X_DEADZONE, X_DEADZONE));
	updateCarrotY(nonlinear_filters::deadzone(
		getYOffsetManual(), -Y_DEADZONE, Y_DEADZONE));
	updateCarrotZ(nonlinear_filters::deadzone(
		getZOffsetManual(), -Z_DEADZONE, Z_DEADZONE));
}

void carrot_control::CarrotControl::updateCarrotX(double xOff)
{
	_carrotPos[0] += xOff;
}

void carrot_control::CarrotControl::updateCarrotX()
{
	updateCarrotX(nonlinear_filters::deadzone(
		getXOffsetManual(), -X_DEADZONE, X_DEADZONE));
}

void carrot_control::CarrotControl::updateCarrotY()
{
	updateCarrotY(nonlinear_filters::deadzone(
		getYOffsetManual(), -Y_DEADZONE, Y_DEADZONE));
}

void carrot_control::CarrotControl::updateCarrotZ()
{
	updateCarrotZ(nonlinear_filters::deadzone(
		getZOffsetManual(), -Z_DEADZONE, Z_DEADZONE));
}

void carrot_control::CarrotControl::updateCarrotY(double yOff)
{
	_carrotPos[1] += yOff;
}

void carrot_control::CarrotControl::updateCarrotZ(double zOff)
{
	_carrotPos[2] += zOff;
}

double carrot_control::CarrotControl::distanceToCarrot()
{
	return sqrt(
		pow((getCurrPosition()[0] - _carrotPos[0]), 2) +
		pow((getCurrPosition()[1] - _carrotPos[1]), 2) +
		pow((getCurrPosition()[2] - _carrotPos[2]), 2));
}

double carrot_control::CarrotControl::carrotDistSquaredX()
{
	return pow((getCurrPosition()[0] - _carrotPos[0]), 2);
}

double carrot_control::CarrotControl::carrotDistSquaredY()
{
	return pow((getCurrPosition()[1] - _carrotPos[1]), 2);
}

double carrot_control::CarrotControl::carrotDistSquaredZ()
{
	return pow((getCurrPosition()[2] - _carrotPos[2]), 2);
}

void carrot_control::CarrotControl::publishPosSp(ros::Publisher& pub)
{
	geometry_msgs::Vector3 mess;
	mess.x = _carrotPos[0];
	mess.y = _carrotPos[1];
	mess.z = _carrotPos[2];
	pub.publish(mess);
}

void carrot_control::CarrotControl::publishVelSp(ros::Publisher& pub)
{
	geometry_msgs::Vector3 mess;
	mess.x = _carrotVel[0];
	mess.y = _carrotVel[1];
	mess.z = _carrotVel[2];;
	pub.publish(mess);
}

void carrot_control::CarrotControl::publishPosMv(ros::Publisher& pub)
{
	geometry_msgs::Vector3 mess;
	mess.x = getCurrPosition()[0];
	mess.y = getCurrPosition()[1];
	mess.z = getCurrPosition()[2];
	pub.publish(mess);
}

void carrot_control::CarrotControl::publishVelMv(ros::Publisher& pub)
{
	geometry_msgs::Vector3 mess;
	mess.x = getCurrVelocity()[0];
	mess.y = getCurrVelocity()[1];
	mess.z = getCurrVelocity()[2];
	pub.publish(mess);
}

void carrot_control::CarrotControl::resetPositionPID()
{
	_posXPID->resetIntegrator();
	_posYPID->resetIntegrator();
	_posZPID->resetIntegrator();
}

void carrot_control::CarrotControl::resetVelocityPID()
{
	_velXPID->resetIntegrator();
	_velYPID->resetIntegrator();
	_velZPID->resetIntegrator();
}

void carrot_control::CarrotControl::calculateAttThrustSp(double dt)
{
	_carrotVel[0] = _posXPID->compute(_carrotPos[0], getCurrPosition()[0], dt);
	_carrotVel[1] = _posYPID->compute(_carrotPos[1], getCurrPosition()[1], dt);
	_carrotVel[2] = _posZPID->compute(_carrotPos[2], getCurrPosition()[2], dt);

	double roll = - _velYPID->compute(_carrotVel[1], getCurrVelocity()[1], dt);
	double pitch = _velXPID->compute(_carrotVel[0], getCurrVelocity()[0], dt);
	double yaw = - getYawSpManual();
	double thrust = _velZPID->compute(_carrotVel[2], getCurrVelocity()[2], dt) + _hoverThrust;

	setAttitudeSp(roll, pitch, yaw);
	setThrustSp(thrust);
}

void carrot_control::CarrotControl::parametersCallback(
		plane_detection_ros::PositionControlParametersConfig& configMsg,
		uint32_t level)
{
	ROS_WARN("CarrotControl::parametersCallback");

	_posYPID->set_kp(configMsg.k_p_xy);
	_posYPID->set_kd(configMsg.k_d_xy);
	_posYPID->set_ki(configMsg.k_i_xy);
	_posYPID->set_lim_high(configMsg.lim_high_xy);
	_posYPID->set_lim_low(configMsg.lim_low_xy);

	_velYPID->set_kp(configMsg.k_p_vxy);
	_velYPID->set_kd(configMsg.k_d_vxy);
	_velYPID->set_ki(configMsg.k_i_vxy);
	_velYPID->set_lim_high(configMsg.lim_high_vxy);
	_velYPID->set_lim_low(configMsg.lim_low_vxy);
	
	_posXPID->set_kp(configMsg.k_p_xy);
	_posXPID->set_kd(configMsg.k_d_xy);
	_posXPID->set_ki(configMsg.k_i_xy);
	_posXPID->set_lim_high(configMsg.lim_high_xy);
	_posXPID->set_lim_low(configMsg.lim_low_xy);

	_velXPID->set_kp(configMsg.k_p_vxy);
	_velXPID->set_kd(configMsg.k_d_vxy);
	_velXPID->set_ki(configMsg.k_i_vxy);
	_velXPID->set_lim_high(configMsg.lim_high_vxy);
	_velXPID->set_lim_low(configMsg.lim_low_vxy);

	_posZPID->set_kp(configMsg.k_p_z);
	_posZPID->set_kd(configMsg.k_d_z);
	_posZPID->set_ki(configMsg.k_i_z);
	_posZPID->set_lim_high(configMsg.lim_high_z);
	_posZPID->set_lim_low(configMsg.lim_low_z);

	_velZPID->set_kp(configMsg.k_p_vz);
	_velZPID->set_kd(configMsg.k_d_vz);
	_velZPID->set_ki(configMsg.k_i_vz);
	_velZPID->set_lim_high(configMsg.lim_high_vz);
	_velZPID->set_lim_low(configMsg.lim_low_vz);

	_hoverThrust = configMsg.hover;
}

void carrot_control::CarrotControl::initializeParameters(ros::NodeHandle& nh)
{
    JoyControl::initializeParameters(nh);
	ROS_WARN("CarrotControl::initializeParameters()");

    _posYPID->initializeParameters(nh, PID_Y_PARAM);
	_velYPID->initializeParameters(nh, PID_VY_PARAM);
	_posXPID->initializeParameters(nh, PID_X_PARAM);
	_velXPID->initializeParameters(nh, PID_VX_PARAM);
	_posZPID->initializeParameters(nh, PID_Z_PARAM);
	_velZPID->initializeParameters(nh, PID_VZ_PARAM);

	bool initialized = 
		nh.getParam(HOVER_PARAM, _hoverThrust);
	ROS_INFO("New hover thrust: %.2f", _hoverThrust);
	if (!initialized)
	{
		ROS_FATAL("CarrotControl::initalizeParameters() - failed to initialize parameters");
		throw std::runtime_error("CarrotControl parameters not properly initialized.");
	}
}

void carrot_control::CarrotControl::setReconfigureParameters(
	plane_detection_ros::PositionControlParametersConfig& config)
{
    ROS_WARN("CarrotControl::setReconfigureParameters");

	config.k_p_xy = _posYPID->get_kp();
	config.k_i_xy = _posYPID->get_ki();
	config.k_d_xy = _posYPID->get_kd();
	config.lim_low_xy = _posYPID->get_lim_low();
	config.lim_high_xy = _posYPID->get_lim_high();
	
	config.k_p_vxy = _velYPID->get_kp();
	config.k_i_vxy = _velYPID->get_ki();
	config.k_d_vxy = _velYPID->get_kd();
	config.lim_low_vxy = _velYPID->get_lim_low();
	config.lim_high_vxy = _velYPID->get_lim_high();
	
	config.k_p_z = _posZPID->get_kp();
	config.k_i_z = _posZPID->get_ki();
	config.k_d_z = _posZPID->get_kd();
	config.lim_low_z = _posZPID->get_lim_low();
	config.lim_high_z = _posZPID->get_lim_high();

	config.k_p_vz = _velZPID->get_kp();
	config.k_i_vz = _velZPID->get_ki();
	config.k_d_vz = _velZPID->get_kd();
	config.lim_low_vz = _velZPID->get_lim_low();
	config.lim_high_vz = _velZPID->get_lim_high();

	config.hover = _hoverThrust;
}

carrot_control::CarrotControl* carrot_control::CarrotControl::getCarrotPointer()
{
	return this;
}