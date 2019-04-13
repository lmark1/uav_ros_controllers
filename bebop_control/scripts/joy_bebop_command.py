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
        rospy.Subscriber("joy", Joy, self.joy_callback)
        rospy.Subscriber("distance", Float64, self.dist_callback)
        rospy.Subscriber("bebop/odometry_gt", Odometry, self.position_callback)
        rospy.Subscriber("/plane_normal", PoseStamped, self.plane_callback)
        self.plane_normal = Vector3(1, 0, 0)
        self.plane_yaw = 0
        self.uav_normal = Vector3(1, 0, 0)
        self.uav_yaw = 0

        self.dist_ref = -1
        self.distance_mv = -1

        # Define mode publisher
        self.mode_pub = rospy.Publisher('bebop/inspection_mode', Int32, queue_size=1)
     

    def plane_callback(self, msg):
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.plane_yaw = math.atan2(2 * (qw * qz + qx * qy), qw * qw
                                     + qx * qx - qy * qy - qz * qz)
        
        # Convert to vector
        self.plane_normal.x = math.cos(self.plane_yaw)
        self.plane_normal.y = math.sin(self.plane_yaw)

        if self.plane_normal.x < 0:
            self.plane_yaw = self.plane_yaw + math.pi


    def dist_callback(self, msg):
        if (self.distance_mv > 0 and msg.data < 0):
            print("BebopCommander.distanceCallback: Invalid distance detected")
            self.deactivate_inspection()
            
        self.distance_mv = msg.data

    def position_callback(self,data):
        self.x_mv = data.pose.pose.position.x
        self.y_mv = data.pose.pose.position.y

        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.orientation.y
        qz = data.pose.pose.orientation.z
        qw = data.pose.pose.orientation.w

        self.uav_yaw = math.atan2(2 * (qw * qz + qx * qy), qw * qw
                                     + qx * qx - qy * qy - qz * qz)
        
        # Convert to vector
        self.uav_normal.x = math.cos(self.uav_yaw)
        self.uav_normal.y = math.sin(self.uav_yaw)
        

    def joy_callback(self, data):

        # Check if any of the buttons are pressed
        if (data.buttons[4] == 1 and self.mode_msg.data == Commander.MANUAL_MODE):
            
            # Check if distance reference is valid
            self.dist_ref = self.distance_mv
            if (self.dist_ref < 0):
                print("BebopCommander: Unable to enter inspection mode.")
                return

            # Set inspection mode
            self.mode_msg.data = Commander.INSPECION_MODE
            print("BebopCommander: Inspection mode activation successful - following dist {}"
                .format(self.dist_ref))
            self.mode_pub.publish(self.mode_msg)

        elif (data.buttons[4] != 1 and self.mode_msg.data == Commander.INSPECION_MODE):
            print("BebopCommander: Inspection mode deactivated.")
            self.deactivate_inspection()

    # Subscribe to teleop msgs
    def cmd_vel_callback(self,data):

        if (self.mode_msg.data == Commander.MANUAL_MODE):
            # Drive in manual mode
            self.UAV_pose.point.x = self.UAV_pose.point.x + 0.5 * data.linear.x
            self.UAV_pose.point.y = self.UAV_pose.point.y + 0.5 * data.linear.y
            self.yaw = self.yaw + 0.01 * data.angular.z 
        else:
            self.UAV_pose.point.x = self.dist_ref
            self.UAV_pose.point.y = 0.5 * data.linear.y
            self.yaw = self.uav_yaw + self.plane_yaw

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
        self.mode_msg.data = Commander.MANUAL_MODE
        self.mode_pub.publish(self.mode_msg)

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