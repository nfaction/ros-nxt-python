#!/usr/bin/env python
# Standard imports needed for NXT and ROS
import roslib; roslib.load_manifest('learning_nxt')
import rospy

# Import the command and sensor framework
import NXTProxy as nxt

# Main
if __name__ == '__main__':
	# Initialize NXT sensor class
	sensors = nxt.NxtSensors()
	# Initialize NXT motor command class
	command = nxt.NXTProxy()

	# Start command module
	command.connect()

	# Sample sensor read
	print 'Top button press:', sensors.emergency_stop

	# Sample commands to the motors (Port, Publish Name, Time (s), Power (0-100)
	command.issue_motor_command('A', 'claw', 1, 10)
	command.issue_motor_command('B', 'left', 1,  100)
	command.issue_motor_command('C', 'right', 1, 100)
	rospy.sleep(1)
	# Shutdown command
	command.disconnect()