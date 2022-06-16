from smbus2 import SMBus, i2c_msg
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
#from sensor_msgs.msg import Joy

from rov_control.msg import gamepad
msg = gamepad()

bus = "/dev/i2c-1"

def callback(data -> gamepad):
    DPadVert = data.Axis[0]
    RJoyHorz = data.Axis[1]
    RJoyVert = data.Axis[2]
    RBumper  = data.Axis[3]
    LJoyHorz = data.Axis[4]
    LJoyVert = data.Axis[5]
    DPadHorz = data.Axis[6]
    LBumper  = data.Axis[7]

    A = data.Buttons[0]
    B = data.Buttons[1]
    LJoyDown = data.Buttons[2]
    RTrigger = data.Buttons[3]
    Start = data.Buttons[4]
    LTrigger = data.Buttons[5]
    RJoyDown = data.Buttons[6]
    Mode = data.Buttons[7]
    Y   = data.Buttons[8]
    X   = data.Buttons[9]
    Select = data.Buttons[10]


""" 
1.↗️   F     2.↖️


6.o          3.o


5.↘️          4.↙️


"""

    #left joystick X is roll
    if(LJoyHorz > 0):
        #Left motor forwards, Right motor downwards

    return


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", gamepad, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()