#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Transform, Quaternion, Vector3Stamped
from math import atan2, cos, sin

class TrajectoryPacker():
    '''
    Packs a single trajectory point into a trajectory message.
    '''

    def __init__(self):
        rospy.Subscriber('trajectory_point', MultiDOFJointTrajectoryPoint, self.ref_sub)
        rospy.Subscriber('odometry', Odometry, self.odom_sub)
        self.odom_msg = Odometry()
        rospy.Subscriber('carrot/status', String, self.status_cb)
        self.pub_odom = True
        self.old_odom = Odometry()
	self.status = "OFF"
        self.traj_pub = rospy.Publisher('trajectory', MultiDOFJointTrajectory, queue_size=1)
	self.dummy_pub = rospy.Publisher('dummy/velocity', Vector3Stamped, queue_size=1)

    def ref_sub(self, msg):
        traj = MultiDOFJointTrajectory()
        traj.points = []
        traj.points.append(msg)
        if not self.status == "OFF":
            self.traj_pub.publish(traj)
        
    def status_cb(self, msg):
        self.status = msg.data
        if msg.data == "OFF":
            traj = MultiDOFJointTrajectory()
            traj.points = []
            # Take it from odometry
            point = MultiDOFJointTrajectoryPoint()
            transform = Transform()
            transform.translation.x = self.odom_msg.pose.pose.position.x
            transform.translation.y = self.odom_msg.pose.pose.position.y
            transform.translation.z = self.odom_msg.pose.pose.position.z
            transform.rotation.x = self.odom_msg.pose.pose.orientation.x
            transform.rotation.y = self.odom_msg.pose.pose.orientation.y
            transform.rotation.z = self.odom_msg.pose.pose.orientation.z
            transform.rotation.w = self.odom_msg.pose.pose.orientation.w
            point.transforms.append(transform)
            traj.points.append(point)
	    self.traj_pub.publish(traj)

    def odom_sub(self, msg):
        self.odom_msg = msg
	
	dx = msg.pose.pose.position.x - self.old_odom.pose.pose.position.x
	dy = msg.pose.pose.position.y - self.old_odom.pose.pose.position.y
	dz = msg.pose.pose.position.z - self.old_odom.pose.pose.position.z
	qx = msg.pose.pose.orientation.x
	qy = msg.pose.pose.orientation.y
	qz = msg.pose.pose.orientation.z
	qw = msg.pose.pose.orientation.w
	yaw = atan2(2 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz)
	newMsg = Vector3Stamped()
	newMsg.vector.x = (cos(yaw) * dx + sin(yaw) * dy) / 0.02
	newMsg.vector.y = (cos(yaw) * dy - sin(yaw) * dx ) / 0.02
	newMsg.vector.z = yaw
	newMsg.header.stamp = rospy.Time.now()
	self.dummy_pub.publish(newMsg)
	self.old_odom = msg

if __name__ == "__main__":
    rospy.init_node('point_to_trajectory')
    tp = TrajectoryPacker()
    rospy.spin()
