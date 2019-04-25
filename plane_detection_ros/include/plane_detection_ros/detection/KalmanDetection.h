#ifndef KALMAN_DETECTION_H
#define KALMAN_DETECTION_H

#include <plane_detection_ros/detection/PlaneDetection.h>
#include <uav_ros_control/KalmanFilter.h>

/**
 * This class is used for filtering measured distance to plane
 * using a Kalman Filter
 */
class KalmanDetection : public PlaneDetection
{

/** Maximum time with no measurements - Kalman filter*/
#define MAX_INVALID_TIME 2

public:
    KalmanDetection();
    virtual ~KalmanDetection();

	/**
	 * Filters current distance using the Kalman filter.
	 *
	 * @param dt - Filter discretization time
	 */
	void filterCurrentDistance(double dt);

	/**
	 * Publish filtered distance velocity as a Float64 ROS message.
	 */
	void publishFiltDistVel(ros::Publisher& pub);

	/**
	 * Publish filtered distance as a Float64 ROS message.
	 */
	void publishFiltDist(ros::Publisher& pub);

    /**
	 * Do all the parameter initialization here.
	 */
	virtual void initializeParameters(ros::NodeHandle& nh) override;

	/**
	 * Callback function used for setting various parameters.
	 */
	virtual void parametersCallback(
        plane_detection_ros::PlaneDetectionParametersConfig& configMsg,
		uint32_t level) override;

	/**
	 * Set reconfigure parameters in the given config object.
	 */
	virtual void setReconfigureParameters(
        plane_detection_ros::PlaneDetectionParametersConfig& config) override;

private:

	/** Reset state of internal variables. */
	void resetState();

	/** Kalman filter object */
    std::unique_ptr<KalmanFilter> _kalmanFilter;

	/** Flag signaling that kalman filter is initialized. */
	bool _kalmanInitialized;

	/** Filtered distance - Kalman Filter output */
	double _filteredDistance;

	/** Filtered distance velocity - Kalman Filter output */
	double _filteredDistanceVel;

	/** Time passed while measurements are invalid. */
	double _timeInvalid;
};

#endif /* KALMAN_DETECTION_H */
