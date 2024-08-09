#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import subprocess
import os
import signal

def start_ros_node(node_package, node_type, node_name):
    try:
        # Define the command to start the ROS node
        command = [
            'rosrun',
            node_package,
            node_type,
        ]

        # Set the environment for ROS
        env = os.environ.copy()
        env['ROS_PACKAGE_PATH'] = '/opt/ros/noetic/share:/opt/ros/noetic/stacks'  # Change if needed
        env['ROS_MASTER_URI'] = 'http://localhost:11311'  # Change if needed
        env['ROS_PYTHON_LOG_CONFIG_FILE'] = '/opt/ros/noetic/etc/ros/python_logging.conf'  # Change if needed

        # Start the ROS node
        process = subprocess.Popen(command, env=env)

        print("Started ROS node '{}' with PID {}".format(node_name, process.pid))
        return process.pid  # Return the process ID

    except Exception as e:
        print("Failed to start ROS node '{}': {}".format(node_name, e))
        
# Function to stop the ROS node
def stop_ros_node(pid, node_name):
    try:
        os.kill(pid, signal.SIGKILL)  # Send the SIGTERM signal to the process
        print("Stopped ROS node '{}' with PID {}".format(node_name, pid))

    except Exception as e:
        print("Failed to stop ROS node '{}': {}".format(node_name, e))

if __name__ == "__main__":
    # Change the following line to start your specific node
    pid = start_ros_node('crawler_controller', 'q_learning_3x3.py', 'Q_Learning')
    if pid is not None:
        print("waiting to stop")
        # Keep the node running for a while (e.g., 10 seconds) for demonstration purposes
        import time
        time.sleep(15)
        print("stopping")

        # Now stop the node
        stop_ros_node(pid, 'q_learning_3x3.py')



