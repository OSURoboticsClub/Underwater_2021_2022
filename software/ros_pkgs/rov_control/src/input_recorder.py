from controller import Controller

PKG = 'input_recorder'
import roslib; roslib.load_manifest(PKG)

import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import Int16MultiArray


def get_inputs():
    