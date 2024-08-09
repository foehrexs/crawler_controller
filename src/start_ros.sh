#! /bin/bash

source /home/mars/rosenv/bin/activate

source /opt/ros/noetic/setup.bash

source /home/mars/catkin_ws/devel/setup.bash


roslaunch crawler_controller q_learning_3x3.launch 
