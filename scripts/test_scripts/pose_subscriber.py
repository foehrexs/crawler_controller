#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist 

def pose_callback(msg):
	rospy.loginfo(msg)
	
if __name__ == '__main__':
	rospy.init_node("Keyboard subscriber")
	sub = rospy.Subscriber("/cmd_vel", Twist, callback=pose_callback)
	
	rospy.loginfo("Node has been started")
	
	rospy.spin()
