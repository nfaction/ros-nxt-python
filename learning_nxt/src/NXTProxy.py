# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010-2012, Matt DePorter.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of Arizona nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

__author__ = 'Matt DePorter'
__copyright__ = 'Copyright (c) 2010-2012 Matt DePorter'
__credits__ = 'Antons Rebguns, Cody Jorgensen, Cara Slutter'

__license__ = 'BSD'
__maintainer__ = 'Matt DePorter'
__email__ = 'matthew.deporter@sista.arizona.edu'

# Python imports
import time
import math
import sys
from threading import Lock
from threading import Thread
from Queue import Queue

# ROS imports
import rospy
import roslib
roslib.load_manifest('learning_nxt')

# NXT Mindstorm motor and sensor imports
from nxt_msgs.msg import Contact
from nxt_msgs.msg import Color
from nxt_msgs.msg import Range
from nxt_msgs.msg import JointCommand
from sensor_msgs.msg import JointState

class MotorObj:
	""" 
	An object to store motor command information.
	Descriptions:
	
	port: 			Which port the motor is connected to on the NXT Brick
	motor_choice: 	The motor's publishing topic name
	duration: 		How long the motor is to run in seconds
	motor_power:	How much force the motor is to have on this action.
	"""
	def __init__(self, port, motor_choice, duration, motor_power):
		""" A contructor to initialize this object. """
		self.port = port
		self.duration = duration
		self.motor_choice = motor_choice
		self.motor_power = motor_power

class NXTProxy():
	"""
	This class's purpose is to provide a multi-threaded command framework for the NXT's motors. 
	In order for commands to be sent to each motor individually, or commands that are to be issued
	at the same time, threading must be used to allow multiple commands to be issued at once.
	"""
	def __init__(self):
		# Initialize Queues for each motor
		self.__packet_queue_A = Queue()
		self.__packet_queue_B = Queue()
		self.__packet_queue_C = Queue()
		# Initiate motor publisher for ROS
		self.motor_state_pub = rospy.Publisher('/joint_command', JointCommand)

	def connect(self):
		"""
		Initialize a thread for each motor and set the running state.
		"""
		self.running = True
		Thread(target=self.__process_packet_queue_A).start()
		Thread(target=self.__process_packet_queue_B).start()
		Thread(target=self.__process_packet_queue_C).start()

	def disconnect(self):
		"""
		Shutdown node and kill all issued commands.
		"""
		self.running = False
		rospy.loginfo('Shutting down node...')
		self.__queue_new_packet('shutdown')

	def issue_motor_command(self, port, motor_choice, duration, motor_power):
		"""
		Create a motor object and queue that "packet"
		"""
		m = MotorObj(port, motor_choice, duration, motor_power)
		self.__queue_new_packet(m)

	def __queue_new_packet(self, packet):
		"""
		If a shutdown command is issued, queue this first, otherwise dump packet in associated queue
		"""
		if packet == 'shutdown':
			self.__packet_queue_A.put_nowait(packet)
			self.__packet_queue_B.put_nowait(packet)
			self.__packet_queue_C.put_nowait(packet)
		elif packet.port == 'A':
			self.__packet_queue_A.put_nowait(packet)
		elif packet.port == 'B':
			self.__packet_queue_B.put_nowait(packet)
		elif packet.port == 'C':
			self.__packet_queue_C.put_nowait(packet)

	def __process_packet_queue_A(self):
		"""
		Process A's packet command issued from user.
		"""
		while self.running:
			# Block until new packet is available
			packet = self.__packet_queue_A.get(True)
			if packet == 'shutdown':
				sys.exit(1)
			self.__publish_command(packet)
			# Sleep to ensure that command finishes in real time
			rospy.sleep(packet.duration)

	def __process_packet_queue_B(self):
		"""
		Process B's packet command issued from user.
		"""
		while self.running:
			# block until new packet is available
			packet = self.__packet_queue_B.get(True)
			if packet == 'shutdown':
				sys.exit(1)
			self.__publish_command(packet)
			# Sleep to ensure that command finishes in real time
			rospy.sleep(packet.duration)

	def __process_packet_queue_C(self):
		"""
		Process C's packet command issued from user.
		"""
		while self.running:
			# block until new packet is available
			packet = self.__packet_queue_C.get(True)
			if packet == 'shutdown':
				sys.exit(1)
			self.__publish_command(packet)
			# Sleep to ensure that command finishes in real time
			rospy.sleep(packet.duration)

	def __publish_command(self, packet):
		"""
		Publish command to ROS, log state to ROS.  Either run motor for specified duration, or start motor to be stopped later
		"""
		rospy.loginfo('%s Motor running.' % str(packet.motor_choice[0].capitalize() + packet.motor_choice[1:]))
		# Publish initial command, to start the motor
		self.motor_state_pub.publish(packet.motor_choice, packet.motor_power)
		# Time motor spins in seconds
		if float(packet.duration) >= 0.0:
			# Let motor run for the time specified by the user.
			rospy.sleep(packet.duration)
			# Stop motor (Brake motor)
			self.motor_state_pub.publish(packet.motor_choice, 0)
			rospy.loginfo('%s Motor has stopped.' % str(packet.motor_choice[0].capitalize() + packet.motor_choice[1:]))
		# Infinite command (-1 signifies that the motor will contiue to run and will not stop until specified)
		elif packet.duration is -1:
			# Allow motor to continue running
			rospy.loginfo('%s Motor is still running.' % str(packet.motor_choice[0].capitalize() + packet.motor_choice[1:]))

class NxtSensors:	
	"""
	This class's purpose is to provide a object based access to the state of each sensor.  This class is also
	responsible for initializing the ROS node and grabbing all data from the NXT brick and storing those values
	in varibles accessable from outside this class.
	"""
	def __init__(self):
		# Start the ROS node
		rospy.init_node('learning_nxt', anonymous=True)
		# Initialize all sensor variables
		self.emergency_stop = False
		self.rear_bump = False
		self.ultrasonic_sensor = 0.0
		self.color_sensor = 0.0
		self.right_motor = 0.0
		self.left_motor = 0.0
		self.claw_motor = 0.0

	def sensor_listener(self):
		"""
		This method subscripes to the NXT nodes publishing to ROS.  These subscriber commands will grab data from
		the NXT's sensors in real time and make that data available to the class.
		"""
		rospy.Subscriber('/color_intensity_sensor', Color, self.color_intensity_sensor_callback)
		rospy.Subscriber('/emergency_stop', Contact, self.emergency_stop_callback)
		rospy.Subscriber('/rear_bump_sensor', Contact, self.rear_bump_sensor_callback)
		rospy.Subscriber('/ultrasonic_sensor', Range, self.ultrasonic_sensor_callback)
		rospy.Subscriber('/joint_states', JointState, self.motor_states_callback)

	""" Subscriber callbacks """
	def color_intensity_sensor_callback(self, msg):
		self.color_sensor = msg.intensity

	def emergency_stop_callback(self, msg):
		self.emergency_stop = msg.contact

	def rear_bump_sensor_callback(self, msg):
		self.rear_bump = msg.contact

	def ultrasonic_sensor_callback(self, msg):
		self.ultrasonic_sensor = msg.range
		# Prototype conversions.  Might make them available to the user.
		range_init = msg.range
		range_centi = range_init*100
		range_mili = range_init*1000
		range_inches = (range_init*100)/2.54
		range_feet = range_inches/12
		

	def motor_states_callback(self, msg):
		# Right Motor
		self.right_motor = msg.position[0]
		# Left Motor
		self.left_motor = msg.position[1]
		# Claw
		self.claw_motor = msg.position[2]
	
		# Prototype conversions.  Might make them available to the user.
		# Right Motor
		right_motor_pos_rad = msg.position[0]
		right_motor_vel_radsec = msg.velocity[0]
		right_motor_eff_Nm = msg.effort[0]
		# Left Motor
		left_motor_pos_rad = msg.position[1]
		left_motor_vel_radsec = msg.velocity[1]
		left_motor_eff_Nm = msg.effort[1]
		# Claw
		claw_pos_rad = msg.position[2]
		claw_vel_radsec = msg.velocity[2]
		claw_eff_Nm = msg.effort[2]

