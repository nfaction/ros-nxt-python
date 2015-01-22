#!/usr/bin/env python

import roslib; roslib.load_manifest("nxt_python")
import rospy
import time
import nxt.locator
from nxt.sensor import *
from nxt.motor import *
from std_msgs.msg import String
from std_msgs.msg import ColorRGBA

rSpeed = 0
lSpeed = 0

def test_sensors(b):
        global rSpeed
        global lSpeed
        
        rospy.init_node('test_sensor')
        #rospy.rate, rospy.Publisher?: publish touch sensor at 10hz
        pubR = rospy.Publisher('touch_right', String)
        pubL = rospy.Publisher('touch_left', String)
        rospy.Subscriber("/nxt_vel", ColorRGBA, callback)
        
        # access through: terminal ("rostopic echo name").  then, try to get inside of processingjs
        tr = TouchSensor(b, PORT_1)
        tl = TouchSensor(b, PORT_2)
        
        mr = Motor(b, PORT_A)
        ml = Motor(b, PORT_B)
        print "This is the touch sensor test. Make sure that your touch sensor is plugged into port 1:"
        print "TOUCH READING:"
        start = rospy.Time.now()
        while rospy.Time.now()<start+rospy.Duration(100.0):
          #Main Loop starts here
          
          #Get and Print out the status of touch sensor R
          touchR = tr.get_sample()
          print 'TOUCH RIGHT:', touchR
          #Get and Print out the status of touch sensor L
          touchL = tl.get_sample()
          print 'TOUCH LEFT:', touchL
          
          #Create a packaged message to publish to ROS about the sensor data
          touchRight = str(touchR)
          touchLeft = str(touchL)
          pubR.publish(touchRight)
          pubL.publish(touchLeft)
          
          #Motor stuff
          mr.update(rSpeed, 0)
          ml.update(lSpeed, 0)
          
          time.sleep(0.01)
          
def callback(data):
        global rSpeed
        global lSpeed
        print "DATA IN for MOTOR"
        rSpeed = data.r
        lSpeed = data.g
        print rSpeed


sock = nxt.locator.find_one_brick()
if sock:
        test_sensors(sock.connect())
        sock.close()
else:
        print 'No NXT bricks found'
