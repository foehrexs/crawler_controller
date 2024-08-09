#!/usr/bin/env python3

# Gibt die Drehzahl in Umdrehungen pro Minute aus

import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Float32

# Setze die Pins für die Encoder-Signale (z. B. A und B)
# Ersetzen Sie diese durch die tatsächlichen Pin-Nummern
pin_a = 17
pin_b = 18

# Konstanten zur Berechnung der Drehzahl
encoder_ticks_per_rev = 360  # Anzahl der Encoder-Impulse pro Umdrehung
sample_rate = 1000  # Abtastfrequenz in Hz (1 kHz)

# Variablen zur Speicherung der Encoder-Impulse
count_a = 0
count_b = 0


# Drehzahl-Publisher
rospy.init_node('encoder_reader')
pub = rospy.Publisher('encoder_rpm', Float32, queue_size=10)

# Zeitpunkt der letzten Messung
prev_time = rospy.Time.now()

# Initialisiere die GPIO-Pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin_a, GPIO.IN)
GPIO.setup(pin_b, GPIO.IN)

# Callback-Funktion für den A-Encoder-Impuls
def callback_a(channel):
    global count_a
    count_a += 1

# Callback-Funktion für den B-Encoder-Impuls
def callback_b(channel):
    global count_b
    count_b += 1

# Registriere die Callback-Funktionen für die GPIO-Pins
GPIO.add_event_detect(pin_a, GPIO.FALLING, callback=callback_a)
GPIO.add_event_detect(pin_b, GPIO.FALLING, callback=callback_b)

# Hauptschleife
while not rospy.is_shutdown():
    # Berechne die Dauer seit der letzten Messung
    curr_time = rospy.Time.now()
    elapsed_time = (curr_time - prev_time).to_sec()

    # Berechne die Drehzahl in Umdrehungen pro Minute (RPM)
    rpm = (count_a + count_b) * 60.0 / (encoder_ticks_per_rev * elapsed_time)

    # Veröffentliche die Drehzahl
    pub.publish(rpm)

    # Zurücksetzen der Impulszähler
    count_a = 0
    count_b = 0

    # Aktualisiere den Zeitpunkt der letzten Messung
    prev_time = curr_time

    # Schlafen, um die Abtastfrequenz einzuhalten
    rospy.sleep(1.0 / sample_rate)
