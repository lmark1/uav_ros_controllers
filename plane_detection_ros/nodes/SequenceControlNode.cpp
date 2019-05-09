#include <ros/ros.h>
#include <plane_detection_ros/control/SequenceControl.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

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
	nh.getParam("/control/sim_mode", simMode);

    std::shared_ptr<SequenceControl> seqControl
        {new SequenceControl(simMode)};
    seqControl->initializeParameters(nh);
    
	// Joy callback
	ros::Subscriber joySub = nh.subscribe("/joy", 1,
		&SequenceControl::joyCb,
		seqControl.get());

    // Define publishers
    ros::Publisher inspectionPub = nh.advertise<std_msgs::Bool>(
        "/inspection/enabled", 1);
    ros::Publisher leftPub = nh.advertise<std_msgs::Bool>(
        "/sequence/left", 1);
    ros::Publisher rightPub = nh.advertise<std_msgs::Bool>(
        "/sequence/right", 1);
    ros::Publisher stepPub = nh.advertise<std_msgs::Float64>(
        "/sequence/step", 1);

    // Setup loop rate
	double rate = 25;
	nh.getParam("/sequence/rate", rate);
	ros::Rate loopRate(rate);
	double dt = 1.0 / rate;
	ROS_INFO("SequenceControlNode: Setting rate to %.2f", rate);

    double timeElapsed = 0;
	bool holdPosition = false;
	double holdTime = 5;
	nh.getParam("/sequence/hold_time", holdTime);
	ROS_INFO("SequenceControlNode: Setting hold time to %.2f", holdTime);

    while (ros::ok())
    {
        ros::spinOnce();

        seqControl->publishSequenceOffset(stepPub);
        seqControl->publishInpsectionMode(inspectionPub);
        seqControl->publishLeftSequence(leftPub);
        seqControl->publishRightSequence(rightPub);

        loopRate.sleep();
    }
}