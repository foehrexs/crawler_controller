#!/usr/bin/env python3
#-*- coding: utf-8 -*-

from flask import Flask, render_template,jsonify, request, url_for, flash, redirect
from werkzeug.exceptions import abort
import subprocess
import os
import signal
import rospy
from std_msgs.msg import String
from flask_sock import Sock

print("default application")

app = Flask(__name__)
app.config['SECRET_KEY'] = 'Kei'
sock = Sock(app)
recent_data = ""

# Global robot state
robot_state = {"running": False, "data": "No data yet"}

def start_robot():
    global robot_state
    global pid
    global pid_encoder
    #print("parameter:")
    print("in the starting realm")
    #print(cnt)
    #cnt = request.form['countdown']
    params = [
        "_param1:={}".format(cnt),
        "_param2:={}".format(lrate),
        "_param3:={}".format(dfactor)
    ]
    print(params)
    pid_encoder = start_ros_node('crawler_controller', 'encoder_left.py', 'Sensoren_node', params)
    pid = start_ros_node('crawler_controller', 'q_learning_3x3.py', 'Q_Learning', params)
    print("after starting the node")
    robot_state["running"] = True
    robot_state["data"] = "Robot started"

def stop_robot():
    global robot_state
    stop_ros_node(pid, 'q_learning_3x3.py')
    stop_ros_node(pid_encoder, 'encoder_left.py')
    robot_state["running"] = False
    robot_state["data"] = "Robot stopped"

def get_robot_data():
    return robot_state["data"]
    
def start_ros_node(node_package, node_type, node_name, params=None):
    try:
        # Define the command to start the ROS node
        command = [
            'rosrun',
            node_package,
            node_type,
        ]
        
        # Add parameters if any
        if params:
            command.extend(params)

        # Set the environment for ROS
        env = os.environ.copy()
        env['ROS_PACKAGE_PATH'] = '/opt/ros/noetic/share:/opt/ros/noetic/stacks:/home/mars/catkin_ws/src'  # Change if needed
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
        
def callback(data):
    global recent_data
    recent_data = data.data


@app.route('/')
def index():
    return render_template('index.html')

@app.route('/steuerung')
def steuerung():
    return render_template('steuerung.html')


@app.route('/start', methods=['POST'])
def start():   
    global cnt
    global lrate
    global dfactor
    print("starting variables input") 
    print("Form data:", request.form)
    print("Request data:", request.data)  # Print raw data received

    cnt = request.form.get('countdown')
    lrate = request.form.get('lrate')
    dfactor = request.form.get('dfactor')
    print("lrate")
    print(lrate)
    print("cnt")
    print(cnt)
    print("dfactor")
    print(dfactor)
    if dfactor is None:
        return jsonify(status="error", message="ldiscount factor parameter missing"), 400
    if lrate is None:
        return jsonify(status="error", message="learning rate parameter missing"), 400
    if cnt is None:
        return jsonify(status="error", message="countdown parameter missing"), 400

    start_robot()
    return jsonify(status="started")


@app.route('/stop', methods=['POST'])
def stop():
    stop_robot()
    return jsonify(status="stopped")

@app.route('/data', methods=['GET'])
def data():
    data = get_robot_data()
    return jsonify(data=data)
    
@sock.route('/ws')
def ws(ws):
    global recent_data
    #recent_data = "a"
    while True:
        # Send the latest data to the connected client
        #recent_data = recent_data + "a"
        ws.send(recent_data)
        rospy.sleep(1)  # Control the update frequency


if __name__ == '__main__':
    rospy.init_node('App_node')
    rospy.Subscriber('graph_data', String, callback)
    from threading import Thread
    flask_thread = Thread(target=app.run, kwargs={'host': '0.0.0.0', 'port': 5000})
    flask_thread.start()
    rospy.spin()
    #app.run(debug=True, host= '0.0.0.0')
