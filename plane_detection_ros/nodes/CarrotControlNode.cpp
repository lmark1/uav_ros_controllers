#include <plane_detection_ros/control/CarrotControl.h>

int main(int argc, char **argv) {

	// Setup the node
	ros::init(argc, argv, "distance_control");
	ros::NodeHandle nh;

	// Change logging level
	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
		ros::console::levels::Debug))
		ros::console::notifyLoggerLevelsChanged();

    // Initialize distance control object
	std::shared_ptr<carrot_control::CarrotControl> carrotControl
		{ new carrot_control::CarrotControl(nh) };

	// Run default carrot control algorithm
	carrot_control::runDefault(*carrotControl, nh);
}