#!/usr/bin/env python3
#-*- coding: utf-8 -*-

from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import sh1106, ssd1306
from PIL import ImageFont, ImageDraw, Image
import rospy
from std_msgs.msg import String

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
DXL_ID_1_MIN_POSITION = 500
DXL_ID_1_MAX_POSITION = 1500
DXL_ID_2_MIN_POSITION = 1200
DXL_ID_2_MAX_POSITION = 2500
DXL_ID_3_MIN_POSITION = 800
DXL_ID_3_MAX_POSITION = 2200

# Control-Table für XL430-W250
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

# Initialisieren der Motoren
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

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
        return 0  # Standardwert hinzugefügt
    elif dxl_error != 0:
        print("Dynamixel error:", packetHandler.getRxPacketError(dxl_error))
        return 0  # Standardwert hinzugefügt
    else:
        return dxl_position


# Funktion zum Bewegen der Motoren
def adjust_motor_position(id, step):
    current_position = read_motor_position(id)
    
    if id == DXL_ID_1:
        min_position = DXL_ID_1_MIN_POSITION
        max_position = DXL_ID_1_MAX_POSITION
    elif id == DXL_ID_2:
        min_position = DXL_ID_2_MIN_POSITION
        max_position = DXL_ID_2_MAX_POSITION
    elif id == DXL_ID_3:
        min_position = DXL_ID_3_MIN_POSITION
        max_position = DXL_ID_3_MAX_POSITION

    desired_position = current_position + step
    desired_position = max(min_position, min(desired_position, max_position))
    
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

# Erweiterung für Tastatur
def keyboard_node():
    pub = rospy.Publisher('keyboard_input', String, queue_size=10)
    while not rospy.is_shutdown():
        key_input = input("Enter a key: ")
        pub.publish(key_input)


# Keyboard Implementierung
def keyboard_callback(data):
    key = data.data
    if key == '1':
        motor_position = read_motor_position(DXL_ID_1)
        print("Motor 1 current position:", motor_position)
        update_oled_display(f"Motor 1: {motor_position}")
    elif key == '2':
        motor_position = read_motor_position(DXL_ID_2)
        print("Motor 2 current position:", motor_position)
        update_oled_display(f"Motor 2: {motor_position}")
    elif key == '3':
        motor_position = read_motor_position(DXL_ID_3)
        print("Motor 3 current position:", motor_position)
        update_oled_display(f"Motor 3: {motor_position}")
        
    # Steuerung von Motor 1
    if key == 'q':
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        desired_position = adjust_motor_position(DXL_ID_1, 200)
        print("Taste Q wurde gedrückt, Motor 1 dreht links..", desired_position)
    elif key == 'w':
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        desired_position = adjust_motor_position(DXL_ID_1, -200)
        print("Taste W wurde gedrückt, Motor 1 dreht rechts..", desired_position)

    # Steuerung von Motor 2
    if key == 'a':
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_2, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        desired_position = adjust_motor_position(DXL_ID_2, 200)
        print("Taste a wurde gedrückt, Motor 2 dreht links..", desired_position)
    elif key == 's':
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_2, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        desired_position = adjust_motor_position(DXL_ID_2, -200)
        print("Taste s wurde gedrückt, Motor 2 dreht rechts..", desired_position)

    # Steuerung von Motor 3
    if key == 'y':
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_3, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        desired_position = adjust_motor_position(DXL_ID_3, 200)
        print("Taste y wurde gedrückt, Motor 3 dreht links..", desired_position)
    elif key == 'x':
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_3, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        desired_position = adjust_motor_position(DXL_ID_3, -200)
        print("Taste x wurde gedrückt, Motor 3 dreht rechts..", desired_position)



if __name__ == '__main__':
    rospy.init_node("move_control")
    sub = rospy.Subscriber("/joy", Joy, callback=joy_callback)
    rospy.Subscriber('keyboard_input', String, keyboard_callback)
    rospy.loginfo("Node has been started!")
    rospy.spin()
