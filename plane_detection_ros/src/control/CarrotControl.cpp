#include <plane_detection_ros/control/CarrotControl.h>
#include <uav_ros_control/PID.h>
#include <uav_ros_control/NonlinearFilters.h>

// Define all parameter paths here
#define PID_X_PARAM "/control/pos_x"
#define PID_Y_PARAM "/control/pos_y"
#define PID_Z_PARAM "/control/pos_z"
#define PID_VX_PARAM "/control/vel_x"
#define PID_VY_PARAM "/control/vel_y"
#define PID_VZ_PARAM "/control/vel_z"
#define HOVER_PARAM "/control/hover"

carrot_control::CarrotControl::CarrotControl(ros::NodeHandle& nh) :
    _posXPID {new PID ("Position - x")},
    _posYPID {new PID ("Position - y")}, 
    _posZPID {new PID ("Position - z")},
    _velXPID {new PID ("Velocity - x")},
    _velYPID {new PID ("Velocity - y")},
    _velZPID {new PID ("Velocity - z")}, 
	_hoverThrust (0),
    control_base::ControlBase(nh), 
	joy_control::JoyControl(nh)
{
	// Define all publishers
	_pubPositionMv = nh.advertise<geometry_msgs::Vector3>("/uav/position", 1);
	_pubVelocityMv = nh.advertise<geometry_msgs::Vector3>("/uav/velocity", 1);
	_pubCarrotPositionSp = nh.advertise<geometry_msgs::Vector3>("/carrot/position", 1);
	_pubCarrotVelocitySp = nh.advertise<geometry_msgs::Vector3>("/carrot/velocity", 1);
	
	// Initialize class parameters
	initializeParameters(nh);

	// Setup dynamic reconfigure server
	plane_detection_ros::PositionControlParametersConfig posConfig;
	setPositionReconfigureParams(posConfig);
	_posConfigServer.updateConfig(posConfig);
	_posParamCallback = boost::bind(
		&carrot_control::CarrotControl::positionParamsCb, this, _1, _2);
	_posConfigServer.setCallback(_posParamCallback);
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
	updateCarrotX();
	updateCarrotY();
	updateCarrotZ();
}

void carrot_control::CarrotControl::updateCarrotX(double xOff)
{
	_carrotPos[0] += xOff;
}

void carrot_control::CarrotControl::updateCarrotX()
{
	updateCarrotX(getXOffsetManual());
}

void carrot_control::CarrotControl::updateCarrotY()
{
	updateCarrotY(getYOffsetManual());
}

void carrot_control::CarrotControl::updateCarrotZ()
{
	updateCarrotZ(getZOffsetManual());
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
		carrotDistSquaredX() +
		carrotDistSquaredY() +
		carrotDistSquaredZ());
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

void carrot_control::CarrotControl::publishPosSp()
{
	geometry_msgs::Vector3 mess;
	mess.x = _carrotPos[0];
	mess.y = _carrotPos[1];
	mess.z = _carrotPos[2];
	_pubCarrotPositionSp.publish(mess);
}

void carrot_control::CarrotControl::publishVelSp()
{
	geometry_msgs::Vector3 mess;
	mess.x = _carrotVel[0];
	mess.y = _carrotVel[1];
	mess.z = _carrotVel[2];;
	_pubCarrotVelocitySp.publish(mess);
}

void carrot_control::CarrotControl::publishPosMv()
{
	geometry_msgs::Vector3 mess;
	mess.x = getCurrPosition()[0];
	mess.y = getCurrPosition()[1];
	mess.z = getCurrPosition()[2];
	_pubPositionMv.publish(mess);
}

void carrot_control::CarrotControl::publishVelMv()
{
	geometry_msgs::Vector3 mess;
	mess.x = getCurrVelocity()[0];
	mess.y = getCurrVelocity()[1];
	mess.z = getCurrVelocity()[2];
	_pubVelocityMv.publish(mess);
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
	double thrust = 
		_velZPID->compute(_carrotVel[2], getCurrVelocity()[2], dt) + _hoverThrust;

	setAttitudeSp(roll, pitch, yaw);
	setThrustSp(thrust);
}

void carrot_control::CarrotControl::positionParamsCb(
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
		ROS_FATAL("CarrotControl::initalizeParameters() - \
			failed to initialize parameters");
		throw std::runtime_error("CarrotControl parameters not properly initialized.");
	}
}

void carrot_control::CarrotControl::setPositionReconfigureParams(
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

void initilizedLoopParameters(ros::NodeHandle& nh, double& rate, bool& simMode)
{
	bool initialized = nh.getParam("/control/rate", rate) &&
		nh.getParam("/control/sim_mode", simMode);
	ROS_INFO("carrot_control::runDefault() - Setting rate to %.2f", rate);
	ROS_INFO_STREAM("carrot_control::runDefault - Simulation mode enabled:" << 
		std::boolalpha << simMode << std::endl);
	if (!initialized)
	{
		ROS_FATAL("carrot_control::runDefault() - unable to load parameters.");
		throw std::runtime_error("Failed to properly initialize parameters.");
	}

}

void carrot_control::runDefault(carrot_control::CarrotControl& cc, ros::NodeHandle& nh)
{
	double rate = 25;
	bool simMode = false;
	initilizedLoopParameters(nh, rate, simMode);

	ros::Rate loopRate(rate);
	double dt = 1.0 / rate;
	
	while (ros::ok())
	{
		ros::spinOnce();
		
		cc.updateCarrot();
		cc.calculateAttThrustSp(dt);

		// Publish setpoint based on the control mode
		if (simMode)
			cc.publishAttitudeSim(cc.getThrustScale());        
		else
			cc.publishAttitudeReal();

		// Publish carrot setpoints
		cc.publishEulerSp();
		cc.publishPosSp();
		cc.publishVelSp();
		cc.publishPosMv();
		cc.publishVelMv();

		loopRate.sleep();
	}
}