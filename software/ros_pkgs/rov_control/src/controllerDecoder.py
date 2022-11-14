#!/usr/bin/env python3
from smbus2 import SMBus, i2c_msg
#import smbus
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from math import sqrt, cos, sin

#To-do: figure out if would improve performance to limit import to just datatype
import numpy as np
#from sensor_msgs.msg import Joy

from rov_control.msg import gamepad
msg = gamepad()

motorControllAddr = 0x50

def callback(data):
    #TO-DO: normalize
    DPadVert = data.Axis[0]
    RJoyHorz = data.Axis[1]
    RJoyVert = data.Axis[2]
    RBumper  = (self.axis_states.values()[3] + 1.0) / 2.0
    LJoyHorz = data.Axis[5]
    LJoyVert = data.Axis[4]
    DPadHorz = data.Axis[6]
    LBumper  = (self.axis_states.values()[7] + 1.0) / 2.0

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
    0.↗   F     1.↖


    5.o          2.o


    4.↘         3.↙


    """
    motors = [0, 0, 0, 0, 0, 0]
    #left joystick X is roll
    vX = float(RJoyHorz)
    vY = float(RJoyVert)

    angle =  float((RBumper - LBumper) * 45.0)

    #if clockwise
    #NOTE: This doesn't rotate the ROV, it just changes values to rotate the motors. This needs to be changed.
    if (angle >= 5.0):
        vXA = float(vX * cos(angle)) - float(vY * sin(angle))
        vYA = float(vX * sin(angle)) + float(vY * cos(angle))
    elif (angle <= -5.0): #counter-clockwise
        vXA = float(vX * cos(angle)) + float(vY * sin(angle))
        vYA = float(-vX * sin(angle)) + float(vY  * cos(angle))
    else:
        vXA = vX
        vYA = vY

    #mag = sqrt((vX ** 2) + (vY ** 2))
    #if(round(mag, 2) != 0):
    #    vX /= mag
    #    vY /= mag


    if(round(LJoyVert, 2) != 0.0):
        motors[5] += LJoyVert
        motors[2] += LJoyVert

    if(round(LJoyHorz, 2) != 0.0):
        motors[5] += -LJoyHorz
        motors[2] += LJoyHorz

    #left joystick Y would be pitch, don't believe we can change pitch though

    #right joystick is X-Y plane
    #X direction
    if(round(vX, 2) != 0.0):
        motors[1] += vXA
        motors[4] += -vXA
        motors[0] += -vXA
        motors[3] += vXA

    #right joystick Y is Y
    if(round(vY, 2) != 0.0):
        motors[0] += -vYA
        motors[1] += -vYA
        motors[3] += vYA
        motors[4] += vYA

    #triggers control altitude
    #motors[2] += RBumper - LBumper
    #motors[5] += RBumper - LBumper

    #print(motors)

    fmotors = [max(min(int(i*127), 127), -127) for i in motors]

    #Byte loses percision but means less data sending, so overall worth it.
    #To-do, ensure separately that this will work.
    values = np.array(fmotors, dtype=np.byte)

    with SMBus(1) as bus:
        msg = i2c_msg.write(motorControllAddr, values)
        bus.i2c_rdwr(msg)

    if A:
        Toggle_Gripper()

    return

def Toggle_Gripper():
    raise NotImplementedError() 

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("gamepad", gamepad, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()