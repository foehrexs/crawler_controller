#!/usr/bin/env python3
#-*- coding: utf-8 -*-

from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import sh1106, ssd1306
from PIL import ImageFont, ImageDraw, Image

import rospy
from sensor_msgs.msg import Joy
from dynamixel_sdk import *

# Konstanten für OLED
serial = i2c(port=1, address=0x3C)
device = sh1106(serial)
oled_font = ImageFont.truetype('FreeSans.ttf',14)

# Konstanten für den Dynamixel-Motor
ADDR_TORQUE_ENABLE = 64
PROTOCOL_VERSION = 2.0
DXL_ID_1 = 1
DXL_ID_2 = 2
DXL_ID_3 = 3
BAUDRATE = 57600
DEVICENAME = '/dev/ttyUSB0'
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
DXL_MINIMUM_POSITION_VALUE = 0
DXL_MAXIMUM_POSITION_VALUE = 2000

# Control-Table für XL430-W250
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

# Initialisieren der Motoren
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Motoren aktivieren 
def enable_all_motors():
    for motor_id in [DXL_ID_1, DXL_ID_2, DXL_ID_3]:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Failed to enable torque for Motor {motor_id}")
        elif dxl_error != 0:
            print(f"Error occurred while enabling torque for Motor {motor_id}: {packetHandler.getRxPacketError(dxl_error)}")
        else:
            print(f"Motor {motor_id} is now stiff (torque enabled).")

if portHandler.openPort():
    print("Port opened")
else:
    print("Failed to open the port")

if portHandler.setBaudRate(BAUDRATE):
    print("Baudrate set to", BAUDRATE)
else:
    print("Failed to set baudrate")

# Aktivieren aller Motoren direkt nach der Port-Initialisierung
enable_all_motors()



# Funktion für Displayausgabe
def update_oled_display(message):
    with canvas(device) as draw:
        draw.rectangle(device.bounding_box, outline="white", fill="black")
        draw.text((10, 10), message, font=oled_font, fill="white")
   
# Funktion zum Lesen der Motorposition        
def read_motor_position(id):
        dxl_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, id, ADDR_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("Dynamixel communication error:", packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("Dynamixel error:", packetHandler.getRxPacketError(dxl_error))
        else:
            return dxl_position

# Funktion zum Bewegen der Motoren
def adjust_motor_position(id, step):
        current_position = read_motor_position(id)
        desired_position = current_position + step
        desired_position = max(DXL_MINIMUM_POSITION_VALUE, min(desired_position, DXL_MAXIMUM_POSITION_VALUE))
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, desired_position)
        return desired_position




# Definition der joy_callback Funktion
def joy_callback(msg: Joy):
    global DXL_ID_1
    global DXL_ID_2
    global DXL_ID_3
    global ADDR_GOAL_POSITION

    # Knöpfe zuweisen
    button_A_pressed = msg.buttons[1]
    button_X_pressed = msg.buttons[0]
    button_Y_pressed = msg.buttons[3]

    # Ausgabe der Motorposition auf Display
    if button_A_pressed:
        motor_position = read_motor_position(DXL_ID_1)
        print("Motor 1 current position:", motor_position)
        update_oled_display(f"Motor 1: {motor_position}")
    if button_X_pressed:
        motor_position = read_motor_position(DXL_ID_2)
        print("Motor 2 current position:", motor_position)
        update_oled_display(f"Motor 2: {motor_position}")
    if button_Y_pressed:
        motor_position = read_motor_position(DXL_ID_3)
        print("Motor 3 current position:", motor_position)
        update_oled_display(f"Motor 3: {motor_position}")
        

    # Ansteuerung der Motoren
    button_ID1_pressed = msg.axes[0]
    if button_ID1_pressed == 1:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        desired_position = adjust_motor_position(DXL_ID_1, 200)
        print("Steuerkreuz links wurde gedrückt, Motor 1 dreht links..", desired_position)
    elif button_ID1_pressed == -1:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        desired_position = adjust_motor_position(DXL_ID_1, -200)
        print("Steuerkreuz rechts wurde gedrückt, Motor 1 dreht rechts..", desired_position)

    button_ID2_pressed = msg.axes[1]
    if button_ID2_pressed == 1:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_2, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        desired_position2 = adjust_motor_position(DXL_ID_2, 200)
        print("Steuerkreuz hoch wurde gedrückt, Motor 2 dreht links.", desired_position2)
    elif button_ID2_pressed == -1:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_2, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        desired_position2 = adjust_motor_position(DXL_ID_2, -200)
        print("Steuerkreuz runter wurde gedrückt, Motor 2 dreht rechts..", desired_position2)

    button_ID3_pressed = msg.axes[5]
    if button_ID3_pressed == 1:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_3, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        desired_position3 = adjust_motor_position(DXL_ID_3, 200)
        print("Joystick-Links, Motor 3 dreht links..", desired_position3)
    elif button_ID3_pressed == -1:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_3, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        desired_position3 = adjust_motor_position(DXL_ID_3, -200)
        print("Joystick-Links, Motor 3 dreht rechts..", desired_position3)


if portHandler.openPort():
    print("Port opened")
else:
    print("Failed to open the port")
if portHandler.setBaudRate(BAUDRATE):
    print("Baudrate set to", BAUDRATE)
else:
    print("Failed to set baudrate")

if __name__ == '__main__':
    rospy.init_node("move_control")
    sub = rospy.Subscriber("/joy", Joy, callback=joy_callback)
    rospy.loginfo("Node has been started!")
    rospy.spin()
