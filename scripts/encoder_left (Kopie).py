#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Int32

# Pins definieren
A_PIN = 16  # Beispiel-Pin für A
B_PIN = 20  # Beispiel-Pin für B

encoder_position = 0

def encoder_callback(channel):
    global encoder_position
    if GPIO.input(A_PIN) == GPIO.input(B_PIN):
        encoder_position -= 1
    else:
        encoder_position += 1

def sensor_node():
    rospy.init_node('left_encoder')  # Knotenname für linken Sensor; ändern Sie für rechten Sensor
    pub = rospy.Publisher('left_encoder_value', Int32, queue_size=10)  # Thema für linken Sensor

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(A_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(B_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # Interrupts zum Erfassen von Flankenänderungen hinzufügen
    GPIO.add_event_detect(A_PIN, GPIO.BOTH, callback=encoder_callback)
    GPIO.add_event_detect(B_PIN, GPIO.BOTH, callback=encoder_callback)

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        pub.publish(encoder_position)
        rate.sleep()

    GPIO.cleanup()

if __name__ == '__main__':
    try:
        sensor_node()
    except rospy.ROSInterruptException:
        pass
