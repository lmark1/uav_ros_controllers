#include <plane_detection_ros/detection/KalmanDetection.h>

KalmanDetection::KalmanDetection():
    _kalmanFilter {new KalmanFilter},
    _kalmanInitialized {false},
    _filteredDistance (NO_PLANE_DETECTED),
    _filteredDistanceVel (0),
    _timeInvalid (0),
    PlaneDetection()
{
}

KalmanDetection::~KalmanDetection()
{   
}

void KalmanDetection::initializeParameters(ros::NodeHandle& nh)
{
    PlaneDetection::initializeParameters(nh);
    ROS_DEBUG("KalmanDetection::initializeParameters()");

    double kalmanNoiseMv;
    double kalmanNoisePos;
    double kalmanNoiseVel;
    bool initialized = 
        nh.getParam("/kalman/noise_mv", kalmanNoiseMv) &&
        nh.getParam("/kalman/noise_pos", kalmanNoisePos) &&
        nh.getParam("/kalman/noise_vel", kalmanNoiseVel);

    _kalmanFilter->setMeasureNoise(kalmanNoiseMv);
    _kalmanFilter->setPositionNoise(kalmanNoisePos);
    _kalmanFilter->setVelocityNoise(kalmanNoiseVel);
    ROS_INFO_STREAM(*_kalmanFilter);

    if (!initialized)
    {
        ROS_FATAL("KalmanDetection::initializeParameters() - parameter initialization failed.");
        throw std::invalid_argument("KalmanDetection parameters not properly set.");
    }
}

void KalmanDetection::parametersCallback(
    plane_detection_ros::PlaneDetectionParametersConfig& configMsg,
    uint32_t level)
{
    PlaneDetection::parametersCallback(configMsg, level);
    ROS_DEBUG("KalmanDetection::parametersCallback");
    _kalmanFilter->setMeasureNoise(configMsg.noise_mv);
    _kalmanFilter->setPositionNoise(configMsg.noise_pos);
    _kalmanFilter->setVelocityNoise(configMsg.noise_vel);
    _kalmanInitialized = configMsg.init_kalman;
    ROS_INFO_STREAM(*_kalmanFilter);
}

void KalmanDetection::setReconfigureParameters(
    plane_detection_ros::PlaneDetectionParametersConfig& config)
{
    PlaneDetection::setReconfigureParameters(config);
	ROS_WARN("KalmanDetection - Reconfigure parameters called.");
    config.noise_mv = _kalmanFilter->getMesaureNoise();
    config.noise_pos = _kalmanFilter->getPositionNoise();
    config.noise_vel = _kalmanFilter->getVelocityNoise();
    config.init_kalman = _kalmanInitialized;
}

void KalmanDetection::filterCurrentDistance(double dt)
{   
    double currDistance = getCurrentDistance();

    // Reset filtered distance if filter is not initialized
    if (!_kalmanInitialized)
    {
        resetState();
    }

    // Check if initialization failed
    if (!_kalmanInitialized && currDistance < 0)
    {
        ROS_WARN("KalmanFilter - Failed to initialize");
        return;
    }

    // Check if initialization should take place
    if (!_kalmanInitialized && currDistance >= 0)
    {
        _kalmanInitialized = true;
        _kalmanFilter->initializePosition(currDistance);
        ROS_WARN("KalmanFilter - Initialized.");
    }

    // Do model update
    _kalmanFilter->modelUpdate(dt);

    // Do measure update if everything is valid
    if (newMeasurementReady() && currDistance > 0)
    {
        ROS_INFO("KalmanFilter - New measurement! update called");
        _kalmanFilter->measureUpdate(currDistance);
        resetNewMeasurementFlag();
        _timeInvalid = 0;
    }
    else
    {
        // Increase time invalid
        ROS_WARN("KalmanFilter - doing only model update");
        _timeInvalid += dt;
    }

    // Check if invalid time reached maximum
    if (_timeInvalid > MAX_INVALID_TIME)
    {
        resetState();
        ROS_FATAL("KalmanFilter - Max invalid time reached.");
        return;
    }

    // Get kalman filter position and velocity
    _filteredDistance = _kalmanFilter->getPosition();
    _filteredDistanceVel = _kalmanFilter->getVelocity();
}

void KalmanDetection::publishFiltDistVel(ros::Publisher& pub)
{
    std_msgs::Float64 outputMessage;
    outputMessage.data = _filteredDistanceVel;
    pub.publish(outputMessage); 
}

void KalmanDetection::publishFiltDist(ros::Publisher& pub)
{
    std_msgs::Float64 outputMessage;
    outputMessage.data = _filteredDistance;
    pub.publish(outputMessage); 
}

void KalmanDetection::resetState()
{
    _kalmanInitialized = false;
    _timeInvalid = 0;
    _filteredDistance = NO_PLANE_DETECTED;
    _filteredDistanceVel = 0;
}