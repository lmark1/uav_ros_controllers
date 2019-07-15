#include <plane_detection_ros/control/DistanceControl.h>
#include <uav_ros_control/filters/NonlinearFilters.h>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <array>

#define DIST_PID_PARAMS "/control/distance"
#define DISTVEL_PID_PARAMS "/control/distance_vel"
#define DIST_SP_DEADZONE 0.01

dist_control::DistanceControl::DistanceControl(
	DistanceControlMode mode, ros::NodeHandle& nh) :
	_inspectIndices 			{new joy_struct::InspectionIndices},
	_distancePID 				{new PID("Distance")},
	_distanceVelPID 			{new PID("Distance vel")},
	_distanceMeasured 			(-1),
	_distanceVelocityMeasured 	(0),
	_distVelSp 					(0),
	_distSp 					(-1),
	_inspectionRequestFailed 	(false),
	_planeYaw 					(0),
	_sequenceStep 				(0),
	_mode 						(mode),
	_currState 					(DistanceControlState::MANUAL),
	_currSeq 					(Sequence::NONE),
	carrot_control::CarrotControl(nh)
{
	// Info messages about node start.
	if (_mode == DistanceControlMode::SIMULATION)
		ROS_INFO("DistanceControl::DistanceControl() - Starting node in simulation mode.");
	else
		ROS_INFO("DistanceControl::DistanceControl() - Starting node in real mode.");	

	// Initialize Class parameters
	initializeParameters(nh);

	// Setup all subscribers
	_subDistance = nh.subscribe("/plane/distance", 1, 
		&dist_control::DistanceControl::distanceCb, this);
	_subDistanceVelocity = nh.subscribe("/plane/distance_vel", 1,
		&dist_control::DistanceControl::distanceVelCb, this);
	_subPlaneNormal = nh.subscribe("/plane/normal", 1,
		&dist_control::DistanceControl::normalCb, this);
	_subSequenceStep = nh.subscribe("/sequence/step", 1,
		&dist_control::DistanceControl::seqStepCb, this);

	// Setup all publishers
	_pubControlState = nh.advertise<std_msgs::Int32>("/control_state", 1);
	_pubDistanceSp = nh.advertise<std_msgs::Float64>("/plane/distance_sp", 1);
	_pubDistanceVelocitySp = nh.advertise<std_msgs::Float64>("/plane/distance_vel_sp", 1); 
	_pubCarrotDistance = nh.advertise<std_msgs::Float64>("/carrot/distance", 1);
	_pubSequenceEnabled = nh.advertise<std_msgs::Bool>("/sequence/enabled", 1);

	// Setup dynamic reconfigure server
	plane_detection_ros::DistanceControlParametersConfig distConfig;
	setDistReconfigureParams(distConfig);
	_distConfigServer.updateConfig(distConfig);
	_distParamCallback =  boost::bind(
		&dist_control::DistanceControl::distParamCb, this, _1, _2);
	_distConfigServer.setCallback(_distParamCallback);
}

dist_control::DistanceControl::~DistanceControl() {
	// TODO Auto-generated destructor stub
}

void dist_control::DistanceControl::distanceCb(const std_msgs::Float64ConstPtr& message)
{
	_distanceMeasured = message->data;
}

void dist_control::DistanceControl::distanceVelCb(const std_msgs::Float64ConstPtr& message)
{
	_distanceVelocityMeasured = message->data;
}

void dist_control::DistanceControl::seqStepCb(const std_msgs::Float64ConstPtr& message)
{
	_sequenceStep = message->data;
}

void dist_control::DistanceControl::normalCb(
	const geometry_msgs::PoseStampedConstPtr& message)
{
	_planeYaw = calculateYaw(
		message->pose.orientation.x,
		message->pose.orientation.y,
		message->pose.orientation.z,
		message->pose.orientation.w);

	// Check in which direction is plane normal facing
	double xComponent = cos(_planeYaw);
	if (xComponent < 0)
		_planeYaw += M_PI;

	_planeYaw = nonlinear_filters::wrapMinMax(_planeYaw, -M_PI, M_PI);
}

void dist_control::DistanceControl::detectSequenceChange()
{
	// Reset carrot position when changing sequence direction
	if (inInspectionState() && (
			leftSeqEnbled() && _currSeq == Sequence::RIGHT ||
			rightSeqEnabled() && _currSeq == Sequence::LEFT ))
	{
		ROS_DEBUG("Inspection sequence changed - reset carrot.");
		setCarrotPosition(
			getCurrPosition()[0],
			getCurrPosition()[1],
			getCurrPosition()[2]);
		_currSeq = _currSeq == Sequence::LEFT ? Sequence::RIGHT : Sequence::LEFT;
	}

	// Determine current sequence
	if (inInspectionState() && leftSeqEnbled())
	{	
		// ROS_INFO("DistanceControl::detectSequenceChange - Left sequence activated.");
		_currSeq = Sequence::LEFT;
	}
	else if (inInspectionState() && rightSeqEnabled())
	{
		// ROS_INFO("DistanceControl::detectSequenceChange - Right sequence activated.");
		_currSeq = Sequence::RIGHT;
	}
	else 
	{
		// ROS_INFO("DistanceControl::detectSequenceChange - sequence deactivated");
		_currSeq = Sequence::NONE;
	}
}

void dist_control::DistanceControl::detectStateChange()
{
	// If we're in inspection mode and received distance is invalid
	// then deactivate inspection mode !
	if (inspectionFailed())
	{
		ROS_FATAL("Inspection mode failed - invalid distance");
		deactivateInspection();
		_inspectionRequestFailed = true;
		return;
	}

	// Check if user wants to go to inspection mode
	if (inspectionRequested() && !_inspectionRequestFailed)
	{
		ROS_DEBUG("Inspection mode requested.");
		// Check if current distance is valid
		if (_distanceMeasured < 0)
		{
			ROS_FATAL("Unable to enter inspection mode.");
			_inspectionRequestFailed = true;
			return;
		}

		ROS_INFO("Inspection activation successful-following distance %.2f",
				_distanceMeasured);
		_distSp = _distanceMeasured;
		_currState = DistanceControlState::INSPECTION;
		setCarrotPosition(
			getCurrPosition()[0],
			getCurrPosition()[1],
			getCurrPosition()[2]);
		return;
	}

	// Check if user wants to go back to manual mode
	if (manualRequested())
	{
		ROS_WARN("Manual mode entered.");
		_inspectionRequestFailed = false;
		deactivateInspection();
	}
}

void dist_control::DistanceControl::deactivateInspection()
{
	_currState = DistanceControlState::MANUAL;
	_currSeq = Sequence::NONE;

	// Reset all PIDs
	_distancePID->resetIntegrator();
	_distanceVelPID->resetIntegrator();
	resetPositionPID();
	resetVelocityPID();

	// Reset Carrot position
	setCarrotPosition(
		getCurrPosition()[0],
		getCurrPosition()[1],
		getCurrPosition()[2]);
	ROS_WARN("Inspection mode deactivated successfully.");
}

void dist_control::DistanceControl::publishState()
{
	// Publish 0 for manual state and 1 for inspection state
	std_msgs::Int32 newMessage;
	newMessage.data = inInspectionState() ? 1 : 0;
	_pubControlState.publish(newMessage);
}

void dist_control::DistanceControl::publishDistVelSp()
{
	std_msgs::Float64 newMessage;
	newMessage.data = _distVelSp;
	_pubDistanceVelocitySp.publish(newMessage);
}

void dist_control::DistanceControl::calculateManualSetpoint(double dt)
{	
	setAttitudeSp(
		-getRollSpManual(), 	//roll
		getPitchSpManual(),		//pitch
		-getYawSpManual());  	//yaw
	setThrustSp(getThrustSpUnscaled());	//thrust
}

void dist_control::DistanceControl::calculateInspectionSetpoint(double dt)
{
	updateCarrot();	
	doDistanceControl(dt);
}

void dist_control::DistanceControl::calculateSequenceSetpoint(double dt)
{
	double yOffset = 0;
	if (_currSeq == Sequence::LEFT)
		yOffset = - _sequenceStep;
	else if (_currSeq == Sequence::RIGHT)
		yOffset = _sequenceStep;

	updateCarrotXY(getXOffsetManual(), yOffset);
	updateCarrotZ();

	doDistanceControl(dt);
	_sequenceStep = 0; // Reset sequence step
}

void dist_control::DistanceControl::doDistanceControl(double dt)
{
	calculateAttThrustSp(dt);

	// update distance setpoint
	_distSp -= nonlinear_filters::deadzone(
		getXOffsetManual(), - DIST_SP_DEADZONE, DIST_SP_DEADZONE);

	// Calculate pitch setpoint using measured distance
	_distVelSp = _distancePID->compute(_distSp, _distanceMeasured, dt);
	double pitch = - _distanceVelPID->compute(_distVelSp, _distanceVelocityMeasured, dt);

	// If in simulation mode treat as YAW RATE, otherwise treat as YAW
	double yaw;
	if (_mode == DistanceControlMode::SIMULATION)
		yaw = _planeYaw * getYawScale();
	else 
		yaw = getUAVYaw() - _planeYaw; 		// TODO: Minus ili plus

	overridePitch(pitch);
	overrideYaw(yaw);
}

void dist_control::DistanceControl::publishDistSp()
{
	std_msgs::Float64 newMessage;
	newMessage.data = _distSp;
	_pubDistanceSp.publish(newMessage);
}

void dist_control::DistanceControl::publishSequenceState()
{	
	std_msgs::Bool newMessage;
	if (_currSeq == Sequence::NONE || !inInspectionState())
		newMessage.data = false;
	else 
		newMessage.data = true;
	_pubSequenceEnabled.publish(newMessage);
}

void dist_control::DistanceControl::publishAttSp()
{	
	// REAL - setpoint while in inspection mode
	if (_mode == DistanceControlMode::REAL && inInspectionState())
	{
		publishAttitudeReal(getAttThrustSp(), 0, MASK_IGNORE_RPY_RATE);
		return;
	}

	// REAL - setpoint while not in inspction mode
	if (_mode == DistanceControlMode::REAL && !inInspectionState())
	{		
		publishAttitudeReal();
		return;
	}

	// SIM - setpoint while in simulation mode
	if (_mode == DistanceControlMode::SIMULATION)
	{
		publishAttitudeSim(getThrustScale());
		return;
	}
}

dist_control::Sequence dist_control::DistanceControl::getSequence()
{
	return _currSeq;
}

bool dist_control::DistanceControl::inInspectionState()
{
	return _currState == DistanceControlState::INSPECTION;
}

bool dist_control::DistanceControl::inspectionRequested()
{
	return inspectionEnabled() &&
			_currState == DistanceControlState::MANUAL;
}

bool dist_control::DistanceControl::inspectionFailed()
{
	return _currState == DistanceControlState::INSPECTION &&
			_distanceMeasured < 0;
}

bool dist_control::DistanceControl::manualRequested()
{
	return !inspectionEnabled() && (
		_currState == DistanceControlState::INSPECTION || 
		_inspectionRequestFailed);
}

void dist_control::DistanceControl::initializeParameters(ros::NodeHandle& nh)
{	
	ROS_WARN("DistanceControl::initializeParameters()");

	_distancePID->initializeParameters(nh, DIST_PID_PARAMS);
	_distanceVelPID->initializeParameters(nh, DISTVEL_PID_PARAMS);
	
	bool initialized = 
		nh.getParam("/joy/detection_state", _inspectIndices->INSPECTION_MODE) &&
		nh.getParam("/joy/left_seq", _inspectIndices->LEFT_SEQUENCE) &&
		nh.getParam("/joy/right_seq", _inspectIndices->RIGHT_SEQUENCE);
	ROS_INFO_STREAM(*_inspectIndices);
	if (!initialized)
	{
		ROS_FATAL("DistanceControl::initializeParameters() - inspection index not set.");
		throw std::runtime_error("DistanceControl parameters are not properly set.");
	}
}

void dist_control::DistanceControl::distParamCb(
	plane_detection_ros::DistanceControlParametersConfig& configMsg,
	uint32_t level)
{
	ROS_WARN("DistanceControl::parametersCallback");

	_distancePID->set_kp(configMsg.k_p_dist);
	_distancePID->set_kd(configMsg.k_d_dist);
	_distancePID->set_ki(configMsg.k_i_dist);
	_distancePID->set_lim_high(configMsg.lim_high_dist);
	_distancePID->set_lim_low(configMsg.lim_low_dist);

	_distanceVelPID->set_kp(configMsg.k_p_vdist);
	_distanceVelPID->set_kd(configMsg.k_d_vdist);
	_distanceVelPID->set_ki(configMsg.k_i_vdist);
	_distanceVelPID->set_lim_high(configMsg.lim_high_vdist);
	_distanceVelPID->set_lim_low(configMsg.lim_low_vdist);
}

void dist_control::DistanceControl::setDistReconfigureParams(
	plane_detection_ros::DistanceControlParametersConfig& config)
{
	ROS_WARN("DistanceControl::setReconfigureParameters");
	
	config.k_p_dist = _distancePID->get_kp();
	config.k_i_dist = _distancePID->get_ki();
	config.k_d_dist = _distancePID->get_kd();
	config.lim_low_dist = _distancePID->get_lim_low();
	config.lim_high_dist = _distancePID->get_lim_high();

	config.k_p_vdist = _distanceVelPID->get_kp();
	config.k_i_vdist = _distanceVelPID->get_ki();
	config.k_d_vdist = _distanceVelPID->get_kd();
	config.lim_low_vdist = _distanceVelPID->get_lim_low();
	config.lim_high_vdist = _distanceVelPID->get_lim_high();
}

bool dist_control::DistanceControl::inspectionEnabled()
{
	if (_mode == DistanceControlMode::SIMULATION)
		return getJoyButtons()[_inspectIndices->INSPECTION_MODE] == 1
			|| leftSeqEnbled()
			|| rightSeqEnabled();
	else
		return getJoyButtons()[_inspectIndices->INSPECTION_MODE] == 0
			|| leftSeqEnbled()
			|| rightSeqEnabled();
}

bool dist_control::DistanceControl::leftSeqEnbled()
{
	if (_mode == DistanceControlMode::SIMULATION)
		return getJoyButtons()[_inspectIndices->LEFT_SEQUENCE] == 1;
	else
		return getJoyAxes()[_inspectIndices->RIGHT_SEQUENCE] > 0.5;
}

bool dist_control::DistanceControl::rightSeqEnabled()
{
	if (_mode == DistanceControlMode::SIMULATION)
		return getJoyButtons()[_inspectIndices->RIGHT_SEQUENCE] == 1;
	else
		return getJoyAxes()[_inspectIndices->RIGHT_SEQUENCE] < -0.5;
}

double dist_control::DistanceControl::distanceToCarrot()
{
	return sqrt(
		carrotDistSquaredY() + 
		carrotDistSquaredZ() +
		pow(_distSp -  _distanceMeasured, 2));
}

void dist_control::DistanceControl::publishDistanceToCarrot()
{
	std_msgs::Float64 newMessage;
	if (_currSeq == Sequence::NONE)
		newMessage.data = -1;
	else 
		newMessage.data = distanceToCarrot();
	_pubCarrotDistance.publish(newMessage);
}

void dist_control::DistanceControl::publishDistanceInfo()
{
	publishState();
	publishDistanceToCarrot();
	publishDistSp();
	publishDistVelSp();
	publishSequenceState();
}

void dist_control::runDefault(
	dist_control::DistanceControl& dc, ros::NodeHandle& nh, bool simMode)
{
	// Setup loop rate
	double rate = 25;
	bool initialized = nh.getParam("/sequence/rate", rate);
	ros::Rate loopRate(rate);
	double dt = 1.0 / rate;
	ROS_INFO("dist_control::runDeafult() - Setting rate to %.2f", rate);
	if (!initialized)
	{
		ROS_FATAL("Failed to initialized loop parameters.");
		throw std::runtime_error("Failed to initialize loop parametrs");
	}
	
	// Start the main loop
	while (ros::ok())
	{
		ros::spinOnce();

		dc.detectStateChange();
		dc.detectSequenceChange();
		
		// Do regular "Manual" Inspection when sequence is not set
		if (dc.inInspectionState() && 
			dc.getSequence() == dist_control::Sequence::NONE)
			dc.calculateInspectionSetpoint(dt);

		// Do "Sequence" Inpsection when sequence is set
		else if (dc.inInspectionState() &&
			dc.getSequence() != dist_control::Sequence::NONE)	
			dc.calculateSequenceSetpoint(dt);			

		// If not in inspection state go to attitude control
		else
			dc.calculateManualSetpoint(dt);
			
		// Publish attitude setpoint
		dc.publishAttSp();

		// Publish other information
		dc.publishDistanceInfo();
		dc.publishEulerSp();
		dc.publishCarrotInfo();

		loopRate.sleep();
	}
}
