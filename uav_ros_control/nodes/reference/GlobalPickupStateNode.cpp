#include "uav_ros_control/reference/GlobalPickupStateMachine.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "brick_pickup_node");
    ros::NodeHandle nh;

    std::shared_ptr<uav_sm::GlobalPickupStateMachine> vssmObj{new uav_sm::GlobalPickupStateMachine(nh)};
    ros::MultiThreadedSpinner spinner(2); // Use 4 threads
    spinner.spin(); // spin() will not return until the node has been shutdown
}