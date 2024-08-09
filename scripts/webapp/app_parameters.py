from flask import Flask, render_template,jsonify, request, url_for, flash, redirect
from werkzeug.exceptions import abort
import subprocess
import os
import signal

print("app_parameters.py")

app = Flask(__name__)
app.config['SECRET_KEY'] = 'Kei'

# Global robot state
robot_state = {"running": False, "data": "No data yet"}

def start_robot():
    global robot_state
    global pid
    params = [
        "_param1:=6",
    ]
    pid = start_ros_node('crawler_controller', 'q_learning_3x3_parameters.py', 'Q_Learning', params)
    robot_state["running"] = True
    robot_state["data"] = "Robot started"

def stop_robot():
    global robot_state
    stop_ros_node(pid, 'q_learning_3x3.py')
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
        return process.pid  # Return the process ID

        print("Started ROS node '{}' with PID {}".format(node_name, process.pid))

    except Exception as e:
        print("Failed to start ROS node '{}': {}".format(node_name, e))
        
# Function to stop the ROS node
def stop_ros_node(pid, node_name):
    try:
        os.kill(pid, signal.SIGKILL)  # Send the SIGTERM signal to the process
        print("Stopped ROS node '{}' with PID {}".format(node_name, pid))

    except Exception as e:
        print("Failed to stop ROS node '{}': {}".format(node_name, e))


@app.route('/')
def index():
    return render_template('index.html')

@app.route('/steuerung')
def steuerung():
    return render_template('steuerung.html')


@app.route('/start', methods=['POST'])
def start():
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

if __name__ == '__main__':
    app.run(debug=True, host= '0.0.0.0')
