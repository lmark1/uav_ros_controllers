#include "uav_ros_control/reference/MasterPickupControl.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "master_pickup_node");
    ros::NodeHandle nh;

    auto masterPickup = std::make_shared<uav_sm::MasterPickupControl>(nh);
    ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    spinner.spin(); // spin() will not return until the node has been shutdown
}