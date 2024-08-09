#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Int32

# Pins definieren
A_PIN = 17  # Beispiel-Pin für A
B_PIN = 27  # Beispiel-Pin für B

encoder_position = 0
previous_a = None

def encoder_callback(channel):
    global encoder_position
    global previous_a

    # Zustände des A- und B-Pins lesen
    a_state = GPIO.input(A_PIN)
    b_state = GPIO.input(B_PIN)

    if previous_a is None:
        previous_a = a_state
        return

    # Überprüfen Sie die Drehrichtung basierend auf der Änderung in A und dem aktuellen Wert von B
    if previous_a == 0 and a_state == 1:
        if b_state == 0:
            encoder_position += 1
        else:
            encoder_position -= 1

    previous_a = a_state

def sensor_node():
    rospy.init_node('right_encoder')
    pub = rospy.Publisher('right_encoder_value', Int32, queue_size=10)

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(A_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(B_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # Hinzufügen von Interrupts zum Erfassen von Flankenänderungen
    GPIO.add_event_detect(A_PIN, GPIO.BOTH, callback=encoder_callback, bouncetime=5)

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

