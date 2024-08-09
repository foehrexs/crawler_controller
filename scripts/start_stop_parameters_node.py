#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import subprocess
import os
import signal

# Function to start the ROS node with parameters
def start_ros_node(node_package, node_type, node_name, params=int):
    try:
        # Base command to start the ROS node
        command = [
            'rosrun',
            node_package,
            node_type,
        ]
        
        # Add parameters if any
        if params:
            command.extend(params)

         #Set the environment for ROS
        env = os.environ.copy()
        #print("start env")
        #print(env)
        #print("end env")
        env['ROS_PACKAGE_PATH'] = '/opt/ros/noetic/share:/opt/ros/noetic/stacks:/home/mars/catkin_ws/src'  # Change if needed
        env['ROS_MASTER_URI'] = 'http://localhost:11311'  # Change if needed
        env['ROS_PYTHON_LOG_CONFIG_FILE'] = '/opt/ros/noetic/etc/ros/python_logging.conf'  # Change if needed

        # Start the ROS node
        process = subprocess.Popen(command, env=env)

        print("Started ROS node '{}' with PID {}".format(node_name, process.pid))
        return process.pid  # Return the process ID

    except Exception as e:
        print("Failed to start ROS node '{}': {}".format(node_name, e))
        return None

# Function to stop the ROS node
def stop_ros_node(pid, node_name):
    try:
        os.kill(pid, signal.SIGTERM)  # Send the SIGTERM signal to the process
        print("Sent SIGTERM to ROS node '{}' with PID {}".format(node_name, pid))

        # Give some time for the process to terminate gracefully
        import time
        time.sleep(5)

        # Check if the process is still running and forcefully terminate if needed
        if is_process_running(pid):
            os.kill(pid, signal.SIGKILL)  # Send the SIGKILL signal to the process
            print("Sent SIGKILL to ROS node '{}' with PID {}".format(node_name, pid))

    except Exception as e:
        print("Failed to stop ROS node '{}': {}".format(node_name, e))

def is_process_running(pid):
    try:
        # Check if the process is running by sending signal 0
        os.kill(pid, 0)
    except OSError:
        return False
    return True

if __name__ == "__main__":
    # Parameters to be passed to the node
    params = [
        "_param1:=value1",
        "_param2:=value2",
        "old_topic:=remapped_topic"
    ]

    # Example: Start the 'my_robot_node' node from the 'my_robot_package' package with parameters
    pid = start_ros_node('crawler_controller', 'node_parameters.py', 'my_robot_node_instance', params)

    if pid is not None:
        # Keep the node running for a while (e.g., 10 seconds) for demonstration purposes
        import time
        time.sleep(10)

        # Now stop the node
        stop_ros_node(pid, 'my_robot_node_instance')
