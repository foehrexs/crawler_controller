#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from dynamixel_sdk import *

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
DXL_MINIMUM_POSITION_VALUE = 500
DXL_MAXIMUM_POSITION_VALUE = 1500

# Control-Table für XL430-W250
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

# Initialisieren von Dynamixel-SDK
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)





def read_motor_position(id):
        dxl_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, id, ADDR_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("Dynamixel communication error:", packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("Dynamixel error:", packetHandler.getRxPacketError(dxl_error))
        else:
            return dxl_position

def joy_callback(msg: Joy):
# Knöpfe zuweisen
    button_A_pressed = msg.buttons[1]
    button_X_pressed = msg.buttons[0]
    button_Y_pressed = msg.buttons[3]

# Ausgabe der Ist-Werte
    if button_A_pressed:
        print("Motor 1 current position:", read_motor_position(DXL_ID_1))
    if button_X_pressed:
        print("Motor 2 current position:", read_motor_position(DXL_ID_2))
    if button_Y_pressed:
        print("Motor 3 current position:", read_motor_position(DXL_ID_3))

# Funktion zum Bewegen der Motoren
    def adjust_motor_position(id, step):
        current_position = read_motor_position(id)
        desired_position = current_position + step
        desired_position = max(DXL_MINIMUM_POSITION_VALUE, min(desired_position, DXL_MAXIMUM_POSITION_VALUE))
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, desired_position)
        return desired_position

    button_ID1_pressed = msg.axes[0]
    if button_ID1_pressed == 1:
        desired_position = adjust_motor_position(DXL_ID_1, 100)
        print("Steuerkreuz links wurde gedrückt, Motor 1 dreht links..", desired_position)
    elif button_ID1_pressed == -1:
        desired_position = adjust_motor_position(DXL_ID_1, -100)
        print("Steuerkreuz rechts wurde gedrückt, Motor 1 dreht rechts..", desired_position)

    button_ID2_pressed = msg.axes[1]
    if button_ID2_pressed == 1:
        desired_position2 = adjust_motor_position(DXL_ID_2, 100)
        print("Steuerkreuz hoch wurde gedrückt, Motor 2 dreht links.", desired_position2)
    elif button_ID2_pressed == -1:
        desired_position2 = adjust_motor_position(DXL_ID_2, -100)
        print("Steuerkreuz runter wurde gedrückt, Motor 2 dreht rechts..", desired_position2)

    button_ID3_pressed = msg.axes[5]
    if button_ID3_pressed == 1:
        desired_position3 = adjust_motor_position(DXL_ID_3, 100)
        print("Joystick-Links, Motor 3 dreht links..", desired_position3)
    elif button_ID3_pressed == -1:
        desired_position3 = adjust_motor_position(DXL_ID_3, -100)
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
