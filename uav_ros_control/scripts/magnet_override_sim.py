#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty
from std_msgs.msg import Float32

class MagnetOverride:

  def __init__(self):
    self.service = rospy.Service('magnet/override_ON', Empty, self.override_cb)
    self.magnet_pub = rospy.Publisher('/magnet_uav/gain', Float32, queue_size=1)

  def override_cb(self, req, resp):
    print("MagnetOverride - turning off magnet")
    offMsg = Float32()
    offMsg.data = 0
    self.magnet_pub.publish(offMsg)
    
    rospy.sleep(3.0)

    print("MagnetOverride - on off magnet")
    onMsg = Float32()
    onMsg.data = 1
    self.magnet_pub.publish(onMsg)
    return True


if __name__ == "__main__":
  rospy.init_node("magnet_override_node")
  mag = MagnetOverride()
  rospy.spin()