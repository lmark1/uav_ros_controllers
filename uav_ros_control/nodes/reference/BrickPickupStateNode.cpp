#include "uav_ros_control/reference/BrickPickupStateMachine.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vs_state_machine_node");
    ros::NodeHandle nh;

    std::shared_ptr<uav_sm::BrickPickupStateMachine> vssmObj{new uav_sm::BrickPickupStateMachine(nh)};
    ros::spin();
}