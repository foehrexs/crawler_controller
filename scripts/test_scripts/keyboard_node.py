#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def keyboard_node():
    # Node initialisieren
    rospy.init_node('keyboard_node')
    pub = rospy.Publisher('keyboard_input', String, queue_size=10)

    while not rospy.is_shutdown():
        key_input = input("Enter a key: ")
        pub.publish(key_input)

if __name__ == '__main__':
    keyboard_node()
