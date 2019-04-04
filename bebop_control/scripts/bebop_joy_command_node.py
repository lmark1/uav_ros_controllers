#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy

class Commander():
    #carrot following controller for z axis requires knowing the position of the UAV 
    

    def __init__(self):
        # Create a publisher for roll pitch yaw cmnds
        self.UAV_pose = Vector3(0.0, 0.0, 1.0)
        self.angle_pub = rospy.Publisher('bebop/angle_ref', Vector3, queue_size=1)
        self.pos_pub = rospy.Publisher('bebop/pos_ref', Vector3, queue_size=1)
        # Initialize message variables.

        # Create a subscriber for color msg
        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)
        rospy.Subscriber("bebop/odometry_gt", Odometry, self.position_callback)
        rate = rospy.Rate(1) # 1hz
        # Main while loop.
        while not rospy.is_shutdown():
            rate.sleep()

    # Subscribe to teleop msgs
    def cmd_vel_callback(self,data):
        
        cmd_yaw = Vector3()
        cmd_yaw.x = 0.0
        cmd_yaw.y = 0.0
        cmd_yaw.z = data.angular.z
        self.angle_pub.publish(cmd_yaw)

        cmd_position = Vector3()
        cmd_position.x = self.UAV_pose.x + 10*data.linear.x #5ms for 20Hz
        cmd_position.y = self.UAV_pose.y + 10*data.linear.y #5ms for 20Hz
        cmd_position.z = self.UAV_pose.z + 10*data.linear.z #5ms for 20Hz
        self.pos_pub.publish(cmd_position)
    
    def position_callback(self,data):
        self.UAV_pose.x = data.pose.pose.position.x
        self.UAV_pose.y = data.pose.pose.position.y
        self.UAV_pose.z = data.pose.pose.position.z
    

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('morusCommander')
    # Go to class functions that do all the heavy lifting.
    # Do error checking.
    try:
        ne = Commander()
    except rospy.ROSInterruptException:
        pass