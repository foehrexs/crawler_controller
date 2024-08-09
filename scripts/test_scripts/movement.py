#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from dynamixel_sdk import *



# Konstanten für den Dynamixel-Motor
ADDR_TORQUE_ENABLE = 64				#
PROTOCOL_VERSION = 2.0 				# Protokoll-Version
DXL_ID_1 = 1
DXL_ID_2 = 2
DXL_ID_3 = 3							# ID des Motors
BAUDRATE = 57600					# Baudrate
DEVICENAME = '/dev/ttyUSB0'			# Serieller Port
TORQUE_ENABLE = 1					# Torque aktivieren
TORQUE_DISABLE = 0					# Torque deaktivieren
DXL_MINIMUM_POSITION_VALUE = 100	# Minimale Motorposition
DXL_MAXIMUM_POSITION_VALUE = 4000	# Maximale Motorposition

# Control-Table für XL430-W250
ADDR_GOAL_POSITION = 116
ADRR_PRESENT_POSITION = 132

# Initialisieren von Dynamixel-SDK
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Callback-Funktion wenn Joystick-Nachrichten empfangen werden
def joy_callback(msg: Joy):
	global DXL_ID_1
	global DXL_ID_2
	global DXL_ID_3
	global ADDR_GOAL_POSITION
		
	# Button-Wert zuweisen
	button_ID1_pressed = msg.axes[0]	# Variable für "LR-gedrückt"
	button_ID2_pressed = msg.axes[1] 	# Variable für Arm
	button_ID3_pressed = msg.axes[5]	# Variable für Fuß

	
# ID 1:		Steuerkreuz L+R
		
	if button_ID1_pressed == 1:
		dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
		# Wenn Left auf Steuerkreuz gedrückt wird, aktivieren des Drehmoment
		desired_position = 200
		print ("Steuerkreuz links wurde gedrückt, Motor 1 dreht links..", desired_position)
		dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID_1, ADDR_GOAL_POSITION, desired_position)
		if dxl_comm_result != COMM_SUCCESS:
			print ("Dynamixel commonication error:", packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error !=0:
			print("Dynamixel error:", packetHandler.getRxPacketError(dxl_error))
			
	if button_ID1_pressed == -1:
		dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
		# Wenn Right auf Steuerkreuz gedrückt wird, aktivieren des Drehmoment
		desired_position = 0
		print ("Steuerkreuz rechts wurde gedrückt, Motor 1 dreht rechts..", desired_position)
		dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID_1, ADDR_GOAL_POSITION, desired_position)
		if dxl_comm_result != COMM_SUCCESS:
			print ("Dynamixel commonication error:", packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error !=0:
			print("Dynamixel error:", packetHandler.getRxPacketError(dxl_error))


# ID_2:		Steuerkreuz U+D
			
	if button_ID2_pressed == 1:
		dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_2, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
		# Wenn Up auf Steuerkreuz gedrückt wird, aktivieren des Drehmoment
		desired_position2 = 100
		print ("Steuerkreuz hoch wurde gedrückt, Motor 2 dreht links.", desired_position2)
		dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID_2, ADDR_GOAL_POSITION, desired_position2)
		if dxl_comm_result != COMM_SUCCESS:
			print ("Dynamixel commonication error:", packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error !=0:
			print("Dynamixel error:", packetHandler.getRxPacketError(dxl_error))
			
	if button_ID2_pressed == -1:
		dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_2, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
		# Wenn Down auf Steuerkreuz gedrückt wird, aktivieren des Drehmoment
		desired_position2 = 0
		print ("Steuerkreuz runter wurde gedrückt, Motor 2 dreht rechts..", desired_position2)
		dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID_2, ADDR_GOAL_POSITION, desired_position2)
		if dxl_comm_result != COMM_SUCCESS:
			print ("Dynamixel commonication error:", packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error !=0:
			print("Dynamixel error:", packetHandler.getRxPacketError(dxl_error))


# ID 3:		Joystick Links U+D

	if button_ID3_pressed == 1:
		dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_3, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
		# Wenn Up mit linkem Stick gedrückt wird, aktivieren des Drehmoment
		desired_position3 = 100
		print ("Joystick-Links, Motor 3 dreht links..", desired_position3)
		dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID_3, ADDR_GOAL_POSITION, desired_position3)
		if dxl_comm_result != COMM_SUCCESS:
			print ("Dynamixel commonication error:", packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error !=0:
			print("Dynamixel error:", packetHandler.getRxPacketError(dxl_error))
			
	if button_ID3_pressed == -1:
		dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_3, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
		# Wenn Down mit linkem Stick gedrückt wird, aktivieren des Drehmoment
		desired_position3 = 0
		print ("Joystick-Links, Motor 3 dreht rechts..", desired_position3)
		dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID_3, ADDR_GOAL_POSITION, desired_position3)
		if dxl_comm_result != COMM_SUCCESS:
			print ("Dynamixel commonication error:", packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error !=0:
			print("Dynamixel error:", packetHandler.getRxPacketError(dxl_error))

# Open the port for communication:
if portHandler.openPort():
	print("Port opened")
else:
	print("Failed to open the port")
	
# Set baudrate and return delay time
if portHandler.setBaudRate(BAUDRATE):
	print("Baudrate set to", BAUDRATE)
else:
	print("Failed to set baudrate")
	
	
	

# Main:	
if __name__ == '__main__':
	
# Initialisieren von ROS-node	
	rospy.init_node("movement")

# Abonieren von ROS-Topic, um Joystick-Befehl zu verarbeiten
	sub = rospy.Subscriber("/joy", Joy, callback=joy_callback)
	
# Info an Konsole
	rospy.loginfo("Node has been started!")
	
# Programm am laufen halten		
	rospy.spin()
	
