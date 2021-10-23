#!/usr/bin/env python

# Author : hardins01

PKG = 'input_recorder'
import roslib; roslib.load_manifest(PKG)

import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import Int16MultiArray

import math
import numpy


#the publisher, writing the thrust values to the converted_thrust topic
pub = rospy.Publisher("converted_thrust", Int16MultiArray, queue_size=10)
published_array = Int16MultiArray()
published_array.data = []


def callback(data):

    #collect the data from the thrust_input topic
    x = data.data[0]
    y = data.data[1]
    magnitude = math.sqrt(x*x + y*y)
    if(magnitude > 1):
        magnitude = 1

    #else-ifs to determine which direction we want to move
    if(x > 0 and y == 0):
        A = int(100 * magnitude)
            B = int(-100 * magnitude)
            C = int(-100 * magnitude)   #EAST
            D = int(100 * magnitude)
    elif(x == y and x > 0):
        A = int(100 * magnitude)
            B = 0
            C = 0                       #NORTHEAST
            D = int(100 * magnitude)
    elif(x == 0 and y > 0):
        A = int(100 * magnitude)
            B = int(100 * magnitude)
            C = int(100 * magnitude)    #NORTH
            D = int(100 * magnitude)
    elif(-x == y and x < 0):
        A = 0
            B = int(100 * magnitude)
            C = int(100 * magnitude)    #NORTHWEST
            D = 0
    elif(x < 0 and y == 0):
        A = int(-100 * magnitude)
            B = int(100 * magnitude)
            C = int(100 * magnitude)    #WEST
            D = int(-100 * magnitude)
    elif(x == y and x < 0):
        A = int(-100 * magnitude)
            B = 0
            C = 0                       #SOUTHWEST
            D = int(-100 * magnitude)
    elif(x == 0 and y < 0):
        A = int(-100 * magnitude)
            B = int(-100 * magnitude)
            C = int(-100 * magnitude)   #SOUTH
            D = int(-100 * magnitude)
    elif(x == -y and x > 0):
        A = 0
            B = int(-100 * magnitude)
            C = int(-100 * magnitude)   #SOUTH
            D = 0
    else:
        A = 0
            B = 0
            C = 0
            D = 0

    #create the ABCD array, publish it to converted_thrust
    published_array.data = [A, B, C, D]
    pub.publish(published_array)


def start():

    #initialize the node, called thrust_converter, which is subscribed to the thrust_input topic
    rospy.init_node("thrust_converter", anonymous=True)
    rospy.Subscriber("thrust_input", Floats, callback)

    #continuously runs until it's forcefully stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        pass
