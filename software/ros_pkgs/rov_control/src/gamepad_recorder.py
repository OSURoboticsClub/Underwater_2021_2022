#!/usr/bin/env python

# Author : hardins01

PKG = 'input_recorder'
import roslib; roslib.load_manifest(PKG)

import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from sensor_msgs.msg import Joy
import math

import numpy


global result   #result is the output of callback, which will be published to the thrust_input topic later

#create the publisher, which publishes to the thrust_input topic
pub = rospy.Publisher("thrust_input", numpy_msg(Floats), queue_size=10)


def callback(data):
    #left stick: NS / EW
    #right stick: up/down   spin L/R

    #axes[0]: left stick (left = 1, right = -1)
    #axes[1]: left stick (up = 1, down = -1)
    #axes[2]: left trigger (pressed = -1, released = 1)
    #axes[3]: right stick (left = 1, right = -1)
    #axes[4]: right stick (up = 1, down = -1)
    #axes[5]: right trigger (pressed = -1, released = 1)
    #axes[6]: dpad (left = 1, right = -1)
    #axes[7]: dpad (up = 1, down = -1)

    #the North-South component of the movement vector
    NS = data.axes[1]

    #the East-West component of the movement vector (East is +)
    EW = -1 * data.axes[0]

    #the magnitude of the vector created by the left stick
    magnitude = math.sqrt(NS*NS + EW*EW)

    #else-ifs to determine which sector of the circle we're in, in the order of the unit circle
    theta = math.degrees(math.atan2(NS, EW))
    if(theta <= 22.5 and theta >= -22.5):
        result = numpy.array([magnitude, 0], dtype=numpy.float32)
    elif(theta < 67.5 and theta > 22.5):
        result = numpy.array([magnitude/math.sqrt(2), magnitude/math.sqrt(2)], dtype=numpy.float32)
    elif(theta <= 112.5 and theta >= 67.5):
        result = numpy.array([0, magnitude], dtype=numpy.float32)
    elif(theta < 157.5 and theta > 112.5):
        result = numpy.array([-magnitude/math.sqrt(2), magnitude/math.sqrt(2)], dtype=numpy.float32)
    elif(theta <= -157.5 or theta >= 157.5):
        result = numpy.array([-magnitude, 0], dtype=numpy.float32)
    elif(theta < -112.5 and theta > -157.5):
        result = numpy.array([-magnitude/math.sqrt(2), -magnitude/math.sqrt(2)], dtype=numpy.float32)
    elif(theta <= -67.5 and theta >= -112.5):
        result = numpy.array([0, -magnitude], dtype=numpy.float32)
    elif(theta < -22.5 and theta > -67.5):
        result = numpy.array([magnitude/math.sqrt(2), -magnitude/math.sqrt(2)], dtype=numpy.float32)
    else:
        result = numpy.array([0, 0], dtype=numpy.float32)

    #print result and publish it to thrust_input topic
    print str(result)
    pub.publish(result)


def start():
    #initialize the gamepad_listener node, subscribed to the Joy topic, runs callback()
    rospy.init_node('gamepad_listener', anonymous=True)
    rospy.Subscriber("joy", Joy, callback)

    #continuously runs until it's forcefully stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        pass 
