#!/usr/bin/env python
import roslib; roslib.load_manifest('ptu_laser_assembler')

import rospy
import PyKDL

from math import sin,cos,pi

from nav_msgs.msg import Odometry

class Convert:
  def __init__(self):
    rospy.init_node('RPYconverter')

    rospy.Subscriber('/icp_error_odom', Odometry, self.HandleOdomMsgs)


  def HandleOdomMsgs(self, data):
    o = data.pose.pose.orientation
    rpy = PyKDL.Rotation.Quaternion(o.x,o.y,o.z,o.w).GetRPY()
    rospy.loginfo("RPY: %f, %f, %f", rpy[0], rpy[1], rpy[2]);



if __name__ == "__main__":
    obj = Convert()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
