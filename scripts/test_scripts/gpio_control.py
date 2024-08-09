#!/usr/bin/env python3

import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Bool

# Setze Pin-Modus auf BCM
GPIO.setmode(GPIO.BCM)

# Definition GPIO Pin 17 (Pysikalisch = Pin 11) soll verwendet werden
output_pin = 17

# Setze Pin als Ausgang
GPIO.setup(output_pin, GPIO.OUT)


# Callback-Funktion, um Pin zu schalten
def pin_callback(data):
	if data.data:
		GPIO.output(output_pin, GPIO.HIGH)
	else:
		GPIO.output(output_pin, GPIO.LOW)

# Main:		
if __name__ == '__main__':
	
# Initialisieren von ROS-node
	rospy.init_node('gpio_control_node')

# Abonieren von ROS-Topic, um Pin zu schalten	
	rospy.Subscriber('pin_control_topic', Bool, pin_callback)

# Programm am laufen halten	
	rospy.spin()

# Pin zurücksetzen, wenn Programm beendet wird	
	GPIO.cleanup()
