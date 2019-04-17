#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PointStamped, Point, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32, Float64

class Commander():
    #carrot following controller for z axis requires knowing the position of the UAV 
    
    MANUAL_MODE = 0
    INSPECION_MODE = 1

    def __init__(self):
        # Create a publisher for roll pitch yaw cmnds
        self.UAV_pose = PointStamped()
        self.UAV_pose.point.x = 0
        self.UAV_pose.point.y = 0
        self.UAV_pose.point.z = 0
        self.yaw = 0

        # Remember the measured position
        self.x_mv = 0
        self.y_mv = 0

        self.mode_msg = Int32()
        self.mode_msg.data = Commander.MANUAL_MODE

        self.angle_pub = rospy.Publisher('bebop/angle_ref', Vector3, queue_size=1)
        self.pos_pub = rospy.Publisher('bebop/pos_ref', Vector3, queue_size=1)

        # Create a subscriber for color msg
        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)
        rospy.Subscriber("bebop/odometry_gt", Odometry, self.position_callback)
        rospy.Subscriber("bebop/inspection_mode", Int32, self.mode_callback)

        self.dist_ref = -1
        self.distance_mv = -1
        self.mode = Commander.MANUAL_MODE


    def mode_callback(self, msg):
        # Detect falling edge
        if (self.mode == Commander.INSPECION_MODE and msg.data == Commander.MANUAL_MODE):
            print("BebopCommander.mode_callback: To manual detected")
            self.deactivate_inspection()

        self.mode = msg.data

    def position_callback(self,data):
        self.x_mv = data.pose.pose.position.x
        self.y_mv = data.pose.pose.position.y        

    # Subscribe to teleop msgs
    def cmd_vel_callback(self,data):

        if (self.mode == Commander.INSPECION_MODE):
            # Only control height in attitude mode
            self.UAV_pose.point.x = 0
            self.UAV_pose.point.y = 0
            self.UAV_pose.point.z = self.UAV_pose.point.z + 0.5 * data.linear.z
            self.publish_position()
            return

        # Drive in manual mode
        self.UAV_pose.point.x = self.UAV_pose.point.x + 0.5 * data.linear.x
        self.UAV_pose.point.y = self.UAV_pose.point.y + 0.5 * data.linear.y
        self.yaw = self.yaw + 0.01 * data.angular.z 

        self.UAV_pose.point.z = self.UAV_pose.point.z + 0.5 * data.linear.z
        self.publish_position()

        cmd_yaw = Vector3()
        cmd_yaw.x = 0.0
        cmd_yaw.y = 0.0
        cmd_yaw.z = self.yaw
        self.angle_pub.publish(cmd_yaw)

    def publish_position(self):
        cmd_position = Vector3()
        cmd_position.x = self.UAV_pose.point.x
        cmd_position.y = self.UAV_pose.point.y
        cmd_position.z = self.UAV_pose.point.z
        self.pos_pub.publish(cmd_position)
      
    def deactivate_inspection(self):
        # Set current reference
        self.UAV_pose.point.x = self.x_mv
        self.UAV_pose.point.y = self.y_mv
        self.publish_position()

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('bebopCommander')
    # Go to class functions that do all the heavy lifting.
    # Do error checking.
    try:
        ne = Commander()

        rate = rospy.Rate(50) # 1hz
        # Main while loop.
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        pass