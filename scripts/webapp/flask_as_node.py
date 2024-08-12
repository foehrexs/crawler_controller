#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy
from flask import Flask

# Initialize Flask app
app = Flask(__name__)

# ROS callback function example
def callback_function(data):
    rospy.loginfo(f"Received data: {data.data}")

# ROS publisher example
def publish_function(pub):
    rate = rospy.Rate(1) # 1 Hz
    while not rospy.is_shutdown():
        msg = "Hello from Flask ROS node"
        pub.publish(msg)
        rate.sleep()

# Define Flask routes
@app.route('/')
def index():
    return "Hello from Flask running as a ROS node!"

def main():
    # Initialize the ROS node
    rospy.init_node('flask_ros_node', anonymous=True)


    # Start Flask app in a separate thread so that it doesn't block ROS
    from threading import Thread
    flask_thread = Thread(target=app.run, kwargs={'host': '0.0.0.0', 'port': 5000})
    flask_thread.start()

    # Keep the node running
    rospy.spin()

if __name__ == "__main__":
    main()
