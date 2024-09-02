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
import socket
#relevant for the display
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import sh1106
from PIL import ImageFont
# OLED-Konstanten
serial = i2c(port=1, address=0x3C)
device = sh1106(serial)
oled_font = ImageFont.truetype('FreeSans.ttf', 14)

print("default application")

app = Flask(__name__)
app.config['SECRET_KEY'] = 'Kei'
sock = Sock(app)
recent_data = ""
ws_connection = None
ws_connection2 = None

# Global robot state
robot_state = {"running": False, "data": "No data yet"}

def update_oled_display(message):
    with canvas(device) as draw:
        draw.rectangle(device.bounding_box, outline="white", fill="black")
        draw.text((5, 5), message, font=oled_font, fill="white")

def get_local_ip():
    # Erstellt einen Dummy-Socket, um die IP-Adresse zu ermitteln
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Verbindet den Socket mit einer öffentlichen Adresse (8.8.8.8:80 ist ein Google DNS)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = '127.0.0.1'
    finally:
        s.close()
    return ip


def start_robot():
    global robot_state
    global pid
    global pid_encoder
    global pid_graph
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
    pid_graph = start_ros_node('crawler_controller', 'data_work.py', 'Graph_node', params)
    print("after starting the node")
    robot_state["running"] = True
    robot_state["data"] = "Robot started"

def stop_robot():
    global robot_state
    stop_ros_node(pid, 'q_learning_3x3.py')
    stop_ros_node(pid_encoder, 'encoder_left.py')
    stop_ros_node(pid_graph, 'data_work.py')
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
    global ws_connection2
    if ws_connection2:
        try:
            # Send the data received from ROS topic to the WebSocket client
            ws_connection2.send(recent_data)
        except Exception as e:
            print(f"Failed to send data over WebSocket: {e}")
            ws_connection2 = None  # Reset the connection if sending fails

def error_callback(data):
    global robot_state
    global error_message
    error_message = data.data
    robot_state["running"] = False
    robot_state["data"] = str(error_message)
    print("in Callback")
    print(error_message)
    global ws_connection
    if ws_connection:
        try:
            # Send the data received from ROS topic to the WebSocket client
            ws_connection.send(error_message)
            print("send the message")
        except Exception as e:
            print(f"Failed to send data over WebSocket: {e}")
            ws_connection = None  # Reset the connection if sending fails


@app.route('/')
def index():
    local_ip = get_local_ip()
    print(f"Die IP-Adresse dieses Computers ist: {local_ip}")
    update_oled_display(str(local_ip)+"\n:5000")
    return render_template('index.html')

@app.route('/steuerung')
def steuerung():
    return render_template('steuerung.html')

@app.route('/graph')
def graph():
    return render_template('graph.html')


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
    #global recent_data
    #recent_data = "a"
    #while True:
        # Send the latest data to the connected client
        #recent_data = recent_data + "a"
        #ws.send(recent_data)
        #rospy.sleep(1)  # Control the update frequency
    global ws_connection
    ws_connection = ws
    print("WebSocket connection opened")
    # Keep the connection open to handle incoming messages (if any)
    while True:
        message = ws.receive()
        if message is None:
            break  # Connection closed, exit loop

    print("WebSocket connection closed")
    ws_connection = None
    
    
@sock.route('/ws2')
def ws2(ws2):
    #global recent_data
    #recent_data = "a"
    #while True:
        # Send the latest data to the connected client
        #recent_data = recent_data + "a"
        #ws.send(recent_data)
        #rospy.sleep(1)  # Control the update frequency
    global ws_connection2
    ws_connection2 = ws2
    print("WebSocket2 connection opened")
    # Keep the connection open to handle incoming messages (if any)
    while True:
        message = ws2.receive()
        if message is None:
            break  # Connection closed, exit loop

    print("WebSocket2 connection closed")
    ws_connection2 = None


if __name__ == '__main__':
    rospy.init_node('App_node')
    rospy.Subscriber('graph_data', String, callback)
    rospy.Subscriber("motor_errors", String, error_callback)
    from threading import Thread
    flask_thread = Thread(target=app.run, kwargs={'host': '0.0.0.0', 'port': 5000})
    flask_thread.start()
    local_ip = get_local_ip()
    update_oled_display(str(local_ip)+"\n:5000")
    rospy.spin()
    #app.run(debug=True, host= '0.0.0.0')
