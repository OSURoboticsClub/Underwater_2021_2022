from smbus2 import SMBus, i2c_msg
#import smbus
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

#To-do: figure out if would improve performance to limit import to just datatype
import numpy as np
#from sensor_msgs.msg import Joy

from rov_control.msg import gamepad
msg = gamepad()

motorControllAddr = 80

def callback(data -> gamepad):
    #TO-DO: normalize
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
    0.↗️   F     1.↖️


    5.o          2.o


    4.↘️         3.↙️


    """
    motors = [0, 0, 0, 0, 0, 0]
    #left joystick X is roll
    if(float(f'{LJoyHorz:.2f}') != 0.0):
        #Left motor forwards, Right motor downwards
        motors[5] += LJoyHorz
        motors[2] += -LJoyHorz

    #left joystick Y would be pitch, don't believe we can change pitch though

    #right joystick X is yaw
    if(float(f'{RJoyHorz:.2f}') != 0.0):
        motors[1] += -RJoyHorz
        motors[4] += -RJoyHorz
        motors[0] += RJoyHorz
        motors[3] += RJoyHorz

    #right joystick Y is forward/backwards
    if(float(f'{RJoyVert:.2f}') != 0.0):
        motors[0] += RJoyVert
        motors[1] += RJoyVert
        motors[3] += -RJoyVert
        motors[4] += -RJoyVert

    #triggers control altitude
    motors[2] += RBumper - LBumper
    motors[5] += RBumper - LBumper

    #Byte loses percision but means less data sending, so overall worth it.
    #To-do, ensure separately that this will work.
    values = np.array(motors, dtype=np.byte)

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

    rospy.Subscriber("chatter", gamepad, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()