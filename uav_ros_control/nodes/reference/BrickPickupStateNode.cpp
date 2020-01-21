#include "uav_ros_control/reference/BrickPickupStateMachine.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "brick_pickup_node");
    ros::NodeHandle nh;

    std::shared_ptr<uav_sm::BrickPickupStateMachine> vssmObj{new uav_sm::BrickPickupStateMachine(nh)};
    ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    spinner.spin(); // spin() will not return until the node has been shutdown
}