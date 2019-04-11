#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PointStamped, Point
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

        self.angle_pub = rospy.Publisher('bebop/angle_ref', Vector3, queue_size=1)
        self.pos_pub = rospy.Publisher('bebop/pos_ref', Vector3, queue_size=1)

        # Create a subscriber for color msg
        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)
        rospy.Subscriber("joy", Joy, self.joy_callback)
        rospy.Subscriber("distance", Float64, self.dist_callback)
        rospy.Subscriber("bebop/odometry_gt", Odometry, self.position_callback)

        self.dist_ref = -1
        self.distance_mv = -1

        # Define mode publisher
        self.mode_pub = rospy.Publisher('bebop/inspection_mode', Int32, queue_size=1)
        self.mode_msg = Int32()
        self.mode_msg.data = Commander.MANUAL_MODE

    def dist_callback(self, msg):
        self.distance_mv = msg.data

    def position_callback(self,data):
        self.x_mv = data.pose.pose.position.x
        self.y_mv = data.pose.pose.position.y

    def joy_callback(self, data):

        # Check if any of the buttons are pressed
        if (data.buttons[4] == 1 and self.mode_msg.data == Commander.MANUAL_MODE):
            
            # Check if distance reference is valid
            self.dist_ref = self.distance_mv
            if (self.dist_ref < 0):
                return

            # Set inspection mode
            self.mode_msg.data = Commander.INSPECION_MODE
            print("BebopCommander: Inspection mode activation successful - following dist {}"
                .format(self.dist_ref))

        elif (data.buttons[4] != 1 and self.mode_msg.data == Commander.INSPECION_MODE):
            print("BebopCommander: Inspection mode deactivated.")
            self.mode_msg.data = Commander.MANUAL_MODE
            self.UAV_pose.point.x = self.x_mv

        self.mode_pub.publish(self.mode_msg)

    # Subscribe to teleop msgs
    def cmd_vel_callback(self,data):

        self.yaw = self.yaw + 0.01 * data.angular.z 
        cmd_yaw = Vector3()
        cmd_yaw.x = 0.0
        cmd_yaw.y = 0.0
        cmd_yaw.z = self.yaw
        self.angle_pub.publish(cmd_yaw)

        if (self.mode_msg.data == Commander.MANUAL_MODE):
            # Drive in manual mode
            self.UAV_pose.point.x = self.UAV_pose.point.x + 0.5 * data.linear.x
            self.UAV_pose.point.y = self.UAV_pose.point.y + 0.5 * data.linear.y
        else:
            self.UAV_pose.point.x = self.dist_ref
            self.UAV_pose.point.y = 0.25 * data.linear.y

        self.UAV_pose.point.z = self.UAV_pose.point.z + 0.5 * data.linear.z
                
        cmd_position = Vector3()
        cmd_position.x = self.UAV_pose.point.x
        cmd_position.y = self.UAV_pose.point.y
        cmd_position.z = self.UAV_pose.point.z
        self.pos_pub.publish(cmd_position)
        
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