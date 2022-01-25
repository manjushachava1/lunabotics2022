#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

import threading
from networktables import NetworkTables

import logging

netTable = None

def callback(data):
	#Converts the joystick data into twist commands
	#Can publish if wanted
	twist = Twist()
	linX = 4*data.axes[1]
	angZ = 4*data.axes[0]

	netTable.putNumber("linearX", linX)
	netTable.putNumber("angularZ", angZ)

	twist.linear.x = linX
	twist.angular.z = angZ
	#pub.publish(twist)
	
	

def run():
	global pub
	#Currently we don't need to publish as the roborio can't accept ros
	#pub = rospy.Publisher('lunabotics/mov', Twist)
	rospy.Subscriber('joy", Joy, callback)
	ropsy.init node('lunaTeleop')
	rospy.spin()

def start():
	logging.basicConfig(level=logging.DEBUG)
	NetworkTables.initialize()
	twst = NetworkTables.getTable("TwistValues")
	global netTable
	netTable = twst
	run()

if name == '__main__':
	start()
