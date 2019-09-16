#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

class TrajectoryPacker():
    '''
    Packs a single trajectory point into a trajectory message.
    '''

    def __init__(self):
        rospy.Subscriber('trajectory_point', MultiDOFJointTrajectoryPoint, self.ref_sub)
        self.traj_pub = rospy.Publisher('trajectory', MultiDOFJointTrajectory, queue_size=1)

    def ref_sub(self, msg):
        traj = MultiDOFJointTrajectory()
        traj.points = []
        traj.points.append(msg)
        self.traj_pub.publish(traj)

if __name__ == "__main__":
    rospy.init_node('point_to_trajectory')
    tp = TrajectoryPacker()
    rospy.spin()
