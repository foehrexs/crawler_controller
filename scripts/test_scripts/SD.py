#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Int32
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import sh1106, ssd1306
from PIL import ImageFont, ImageDraw, Image

# Pin-Nummern für die vier Rechtecksignale und zwei Indexsignale
signal_pin_left_1 = 16
signal_pin_left_2 = 20
index_pin_left = 21  # Zum Beispiel

signal_pin_right_1 = 17
signal_pin_right_2 = 27
index_pin_right = 22  # Zum Beispiel

prev_time_left = 0
prev_time_right = 0
angle_left = 0
angle_right = 0
revolutions_left = 0
revolutions_right = 0

# I2C Verbindung erstellen
serial = i2c(port=1, address=0x3C)
device = sh1106(serial)
oled_font = ImageFont.truetype('FreeSans.ttf', 10)

# Start Nachricht
def show_ready_message():
    with canvas(device) as draw:
        draw.rectangle(device.bounding_box, outline="white", fill="black")
        draw.text((30, 20), "Ready", font=oled_font, fill="white")

def gpio_callback_left(channel):
    global prev_time_left, angle_left
    current_time = rospy.get_time()
    time_diff = current_time - prev_time_left

    if GPIO.input(signal_pin_left_1) == GPIO.HIGH:
        angle_left = (360 * time_diff) / 2

    prev_time_left = current_time
    update_oled_display()

def gpio_callback_right(channel):
    global prev_time_right, angle_right
    current_time = rospy.get_time()
    time_diff = current_time - prev_time_right

    if GPIO.input(signal_pin_right_1) == GPIO.HIGH:
        angle_right = (360 * time_diff) / 2

    prev_time_right = current_time
    update_oled_display()

def index_callback_left(channel):
    global revolutions_left
    revolutions_left += 1

def index_callback_right(channel):
    global revolutions_right
    revolutions_right += 1

def update_oled_display():
    with canvas(device) as draw:
        draw.rectangle(device.bounding_box, outline="white", fill="black")
        draw.text((10, 2), f"Links: {angle_left:.1f} Grad", font=oled_font, fill="white")
        draw.text((10, 14), f"{revolutions_left} Umdrehungen", font=oled_font, fill="white")
        draw.text((10, 28), f"Rechts: {angle_right:.1f} Grad", font=oled_font, fill="white")
        draw.text((10, 40), f"{revolutions_right} Umdrehungen", font=oled_font, fill="white")

def main():
    GPIO.setmode(GPIO.BCM)
    for pin in [signal_pin_left_1, signal_pin_left_2, signal_pin_right_1, signal_pin_right_2, index_pin_left, index_pin_right]:
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    GPIO.add_event_detect(signal_pin_left_1, GPIO.BOTH, callback=gpio_callback_left)
    GPIO.add_event_detect(signal_pin_right_1, GPIO.BOTH, callback=gpio_callback_right)
    GPIO.add_event_detect(index_pin_left, GPIO.RISING, callback=index_callback_left)
    GPIO.add_event_detect(index_pin_right, GPIO.RISING, callback=index_callback_right)

    rospy.init_node('angle_reader_node')

    # Zeigt "Ready" auf dem OLED-Display an
    show_ready_message()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        GPIO.cleanup()

if __name__ == '__main__':
    main()

