#include <ros/ros.h>
#include <plane_detection_ros/control/SequenceControl.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

int main(int argc, char **argv) 
{
   	// Setup the node
	ros::init(argc, argv, "sequence_control");
	ros::NodeHandle nh;
	// Change logging level
	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
		ros::console::levels::Debug))
		ros::console::notifyLoggerLevelsChanged();
        
    // Check if sim mode or real
	bool simMode = false;
	bool initialized = nh.getParam("/control/sim_mode", simMode);

    std::shared_ptr<SequenceControl> seqControl
        {new SequenceControl(simMode)};
    seqControl->initializeParameters(nh);
    
	// Joy callback
	ros::Subscriber joySub = nh.subscribe("/sequence/enabled", 1,
		&SequenceControl::sequenceCb,
		seqControl.get());

    ros::Publisher stepPub = nh.advertise<std_msgs::Float64>(
        "/sequence/step", 1);

    // TODO: Make sure same rate is used for distance control and sequence control
    // Setup loop rate
	double rate = 25;
	nh.getParam("/sequence/rate", rate);
	ros::Rate loopRate(rate);
	double dt = 1.0 / rate;
	ROS_INFO("SequenceControlNode: Setting rate to %.2f", rate);

    double timeElapsed = 0;
    double distTravelled = 0;
	bool holdPosition = false;
	double holdTime = 5;
    double maxStep = 1;
	initialized = initialized && nh.getParam("/sequence/hold_time", holdTime);
    initialized = initialized && nh.getParam("/sequence/seq_step", maxStep);
	ROS_INFO("SequenceControlNode: Setting hold time to %.2f", holdTime);
    ROS_INFO("SequenceControlNode: Setting max step to %.2f", maxStep);

    // Initialize override service here
	ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>(
		"magnet/override_ON");
	std_srvs::Empty emptyMessage;
    
    if (!initialized)
    {
        ROS_FATAL("SequenceControlNode: failed to initialized parameters.");
        return 1;
    }

    while (ros::ok())
    {
        ros::spinOnce();
        if (!holdPosition)
            seqControl->publishSequenceOffset(stepPub);
        else
            seqControl->publishSequenceOffset(stepPub, 0);
        
        // Check if sequence is active
        if (!seqControl->sequenceActive())
        {
            distTravelled = 0;
            timeElapsed = 0;
            holdPosition = false;
            loopRate.sleep();
            continue;
        }

        // Update travelled distance
        if (distTravelled < maxStep)
        {
            ROS_INFO("SequenceNode - distance travelled: %.2f", distTravelled);
            distTravelled += seqControl->getSequenceStep();
            loopRate.sleep();
            continue;
        }

        // Hold position after this point
        holdPosition = true;

        // MaxStep distance is travelled, sleep now!
        if (timeElapsed < holdTime)
        {
            ROS_INFO("SequenceNode - time elapsed: %.2f", timeElapsed);
            timeElapsed += dt;
            loopRate.sleep();
            continue;
        }

        ROS_WARN("SequenceNode - Calling override service");
        // Finished waiting - call service !
        client.call(emptyMessage);
        ROS_WARN("SequenceNode - Override called.");

        // Reset and continue
        timeElapsed = 0;
        distTravelled = 0;
        holdPosition = false;
        loopRate.sleep();
    }
}