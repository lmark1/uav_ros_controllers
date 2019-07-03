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
	joy_control::JoyControl(nh),
	positionHold(false)
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

	// Initialize position reference subscriber
	_subPositionRef = nh.subscribe("/uav/pos_ref", 100, 
		&carrot_control::CarrotControl::positionRefCb, this);

	// Initialize position hold service
	_servicePoisitionHold = nh.advertiseService(
			"/uav/position_hold",
			&carrot_control::CarrotControl::positionHoldCb,
			this);
}

carrot_control::CarrotControl::~CarrotControl()
{
}

bool carrot_control::CarrotControl::positionHoldCb(std_srvs::Empty::Request& request,
	std_srvs::Empty::Response& response)
{
	if (!positionHold)
	{
		ROS_WARN("CarrotControl() - Position hold enabled");
		setCarrotPosition(
			getCurrPosition()[0],
			getCurrPosition()[1],
			getCurrPosition()[2]
		);
		positionHold = true;
	}

	return true;
}

void carrot_control::CarrotControl::positionRefCb(const geometry_msgs::Vector3ConstPtr& posMsg)
{
	if (positionHold)
	{
		ROS_INFO("Position reference recieved - [%.2f, %.2f, %.2f]", 
			posMsg->x, posMsg->y, posMsg->z);
		setCarrotPosition(posMsg->x, posMsg->y, posMsg->z);
	}
	else
	{
		ROS_FATAL("Position reference recieved, but position hold disabled.");
	}
}

void carrot_control::CarrotControl::setCarrotPosition(double x, double y, double z)
{
	_carrotPos[0] = x;
	_carrotPos[1] = y;
	_carrotPos[2] = z;
}

void carrot_control::CarrotControl::updateCarrot()
{
	// Disable Position hold if carrot inputs exist
	if (positionHold && (
		abs(getXOffsetManual()) > 0 || abs(getYOffsetManual()) > 0 || 
		abs(getZOffsetManual()) > 0))
	{
		ROS_WARN("Position hold disabled - resetting carrot position");
		setCarrotPosition(
			getCurrPosition()[0],
			getCurrPosition()[1],
			getCurrPosition()[2]
		);
		positionHold = false;
	}

	// Update carrot unless in position hold
	if (!positionHold)
	{
		updateCarrotXY();
		updateCarrotZ();
	}
}

void carrot_control::CarrotControl::updateCarrotXY()
{
	updateCarrotXY(getXOffsetManual(), getYOffsetManual());
}

void carrot_control::CarrotControl::updateCarrotZ()
{
	updateCarrotZ(getZOffsetManual());
}

void carrot_control::CarrotControl::updateCarrotXY(double xOff, double yOff)
{
	_carrotPos[0] += cos(-getUAVYaw()) * xOff + sin(-getUAVYaw()) * yOff;
	_carrotPos[1] += cos(-getUAVYaw()) * yOff - sin(-getUAVYaw()) * xOff;
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

	// Decouple roll and pitch w.r.t. yaw
	setAttitudeSp( 
		cos(getUAVYaw()) * roll + sin(getUAVYaw()) * pitch, 
		cos(getUAVYaw()) * pitch - sin(getUAVYaw()) * roll, 
		yaw);
	
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

void carrot_control::CarrotControl::publishCarrotInfo()
{
	publishPosSp();
	publishVelSp();
	publishPosMv();
	publishVelMv();
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
		cc.publishCarrotInfo();
		
		loopRate.sleep();
	}
}

void carrot_control::attitudeControl(carrot_control::CarrotControl& cc, ros::NodeHandle& nh)
{
	double rate = 25;
	bool simMode = false;
	initilizedLoopParameters(nh, rate, simMode);

	ros::Rate loopRate(rate);
	double dt = 1.0 / rate; 

	while (ros::ok())
	{
		ros::spinOnce();
		
		// Get attitude setpoint from Joy message
		cc.setAttitudeSp(
			- cc.getRollSpManual(), 	//roll
			cc.getPitchSpManual(),		//pitch
			- cc.getYawSpManual());  	//yaw
		cc.setThrustSp(cc.getThrustSpUnscaled());	//thrust

		// Publish attitude setpoint message
		cc.publishAttitudeReal();	
		cc.publishAttitudeSim(cc.getThrustScale());

		loopRate.sleep();
	}
}

void carrot_control::runDefaultFutaba(carrot_control::CarrotControl& cc, ros::NodeHandle& nh)
{
	double rate = 25;
	bool simMode = false;
	initilizedLoopParameters(nh, rate, simMode);

	ros::Rate loopRate(rate);
	double dt = 1.0 / rate; 
	bool carrot_enabled = false;

	while (ros::ok())
	{
		ros::spinOnce();

		if (!carrot_enabled && cc.getJoyButtons()[5] == 0)
		{
			ROS_INFO("Enabling carrot control! - setting position");
			carrot_enabled = true;
			cc.setCarrotPosition(
				cc.getCurrPosition()[0],
				cc.getCurrPosition()[1],
				cc.getCurrPosition()[2]
			);
		}
		else if (cc.getJoyButtons()[5] == 1)
		{
			ROS_INFO("Carrot disabled.");
			carrot_enabled = false;
		}

		if (carrot_enabled)
		{
			cc.updateCarrot();
			cc.calculateAttThrustSp(dt);
			cc.publishAttitudeReal();
		}
		else
		{
			// Get attitude setpoint from Joy message
			cc.setAttitudeSp(
				- cc.getRollSpManual(), 	//roll
				cc.getPitchSpManual(),		//pitch
				- cc.getYawSpManual());  	//yaw
			cc.setThrustSp(cc.getThrustSpUnscaled());	//thrust

			// Publish attitude setpoint message
			cc.publishAttitudeReal();	
			cc.publishAttitudeSim(cc.getThrustScale());
		}
		
		// Publish carrot setpoints
		cc.publishEulerSp();
		cc.publishCarrotInfo();

		loopRate.sleep();
	}
}
