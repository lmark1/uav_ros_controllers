#include <plane_detection_ros/control/DistanceControl.h>

int main(int argc, char **argv) 
{
	// Setup the node
	ros::init(argc, argv, "distance_control");
	ros::NodeHandle nh;
	// Change logging level
	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
		ros::console::levels::Debug))
		ros::console::notifyLoggerLevelsChanged();

	// Check if sim mode or real
	bool simMode = false;
	bool initialized = nh.getParam("/control/sim_mode", simMode);
	dist_control::DistanceControlMode mode = 
		((simMode) ? 
			dist_control::DistanceControlMode::SIMULATION : 
			dist_control::DistanceControlMode::REAL);
	// Initialize distance control object
	std::shared_ptr<dist_control::DistanceControl> distanceControl
		{ new dist_control::DistanceControl(mode, nh) };

	// Run default distance control
	dist_control::runDefault(*distanceControl, nh, simMode);	
}





