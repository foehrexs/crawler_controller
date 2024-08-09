#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy

def main():
    rospy.init_node('my_robot_node')

    # Get parameters
    param1 = rospy.get_param('~param1', 'default_value1')
    param2 = rospy.get_param('~param2', 'default_value2')

    # Log the parameters
    rospy.loginfo("Parameter 1: {}".format(param1))
    rospy.loginfo("Parameter 2: {}".format(param2))

    # Remapped topics (example usage)
    #pub = rospy.Publisher('remapped_topic', rospy.msg.String, queue_size=10)
    #sub = rospy.Subscriber('remapped_topic', rospy.msg.String, callback)

    # Example loop
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        rospy.loginfo("Node is running with parameters: %s, %s", param1, param2)
        #print("Node is running with parameters: %s, %s", param1, param2)
        rate.sleep()

def callback(msg):
    rospy.loginfo("Received message: %s", msg.data)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

