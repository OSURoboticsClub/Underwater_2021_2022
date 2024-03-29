#!/usr/bin/env python2.7
# This is just for testing controller reading and decoding, not for actual source code
# https://gist.github.com/rdb/8864666
#  Released by rdb under the Unlicense (unlicense.org)
# Based on information from:
# https://www.kernel.org/doc/Documentation/input/joystick-api.txt

import os, struct, array, time
from math import sqrt, cos, sin
from fcntl import ioctl

# These constants were borrowed from linux/input.h

global result   #result is the output of callback, which will be published to the thrust_input topic later
#pub = rospy.Publisher("thrust_input", gamepad, queue_size=10)


button_names = {
    0x120 : 'trigger',
    0x121 : 'thumb',
    0x122 : 'thumb2',
    0x123 : 'top',
    0x124 : 'top2',
    0x125 : 'pinkie',
    0x126 : 'base',
    0x127 : 'base2',
    0x128 : 'base3',
    0x129 : 'base4',
    0x12a : 'base5',
    0x12b : 'base6',
    0x12f : 'dead',
    0x130 : 'a',
    0x131 : 'b',
    0x132 : 'c',
    0x133 : 'x',
    0x134 : 'y',
    0x135 : 'z',
    0x136 : 'tl',
    0x137 : 'tr',
    0x138 : 'tl2',
    0x139 : 'tr2',
    0x13a : 'select',
    0x13b : 'start',
    0x13c : 'mode',
    0x13d : 'thumbl',
    0x13e : 'thumbr',

    0x220 : 'dpad_up',
    0x221 : 'dpad_down',
    0x222 : 'dpad_left',
    0x223 : 'dpad_right',

    # XBox 360 controller uses these codes.
    0x2c0 : 'dpad_left',
    0x2c1 : 'dpad_right',
    0x2c2 : 'dpad_up',
    0x2c3 : 'dpad_down',
}

axis_names = {
    0x00 : 'x',
    0x01 : 'y',
    0x02 : 'z',
    0x03 : 'rx',
    0x04 : 'ry',
    0x05 : 'rz',
    0x06 : 'throttle',
    0x07 : 'rudder',
    0x08 : 'wheel',
    0x09 : 'gas',
    0x0a : 'brake',
    0x10 : 'hat0x',
    0x11 : 'hat0y',
    0x12 : 'hat1x',
    0x13 : 'hat1y',
    0x14 : 'hat2x',
    0x15 : 'hat2y',
    0x16 : 'hat3x',
    0x17 : 'hat3y',
    0x18 : 'pressure',
    0x19 : 'distance',
    0x1a : 'tilt_x',
    0x1b : 'tilt_y',
    0x1c : 'tool_width',
    0x20 : 'volume',
    0x28 : 'misc',
}



class Controller:
    def __init__(self):
        self.axis_states = {}
        self.button_states = {}
        self.axis_map = []
        self.button_map = []
        self.jsdev = open('/dev/input/js0', 'rb')
        # Get the device name.
        #buf = bytearray(63)
        buf = array.array('c', ['\0'] * 64)
        ioctl(self.jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
        js_name = buf.tostring()
        print('Device name: %s' % js_name)

        # Get number of axes and buttons.
        buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a11, buf) # JSIOCGAXES
        self.num_axes = buf[0]

        buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
        self.num_buttons = buf[0]

        # Get the axis map.
        buf = array.array('B', [0] * 0x40)
        ioctl(self.jsdev, 0x80406a32, buf) # JSIOCGAXMAP

        for axis in buf[:self.num_axes]:
            axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
            self.axis_map.append(axis_name)
            self.axis_states[axis_name] = 0.0

        # Get the button map.
        buf = array.array('H', [0] * 200)
        ioctl(self.jsdev, 0x80406a34, buf) # JSIOCGBTNMAP

        for btn in buf[:self.num_buttons]:
            btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
            self.button_map.append(btn_name)
            self.button_states[btn_name] = 0

    def update(self):
        evbuf = self.jsdev.read(8)
        if evbuf:
            t, value, type, number = struct.unpack('IhBB', evbuf)
            if type & 0x01:
                button = self.button_map[number]
                if button:
                    self.button_states[button] = value

            if type & 0x02:
                axis = self.axis_map[number]
                #if axis:
                fvalue = value / 32767.0
                self.axis_states[axis] = fvalue
            #print(self.axis_states.values())
            #print(self.button_states.values())
    def get_axis_states(self):
        return self.axis_states.values()
    def get_button_states(self):
        return self.button_states.values()
    def callback(self):
        #TO-DO: normalize
        #print(self.get_axis_states())
        DPadVert = self.axis_states.values()[0]
        RJoyHorz = self.axis_states.values()[1]
        RJoyVert = self.axis_states.values()[2]
        RBumper  = (self.axis_states.values()[3] + 1.0) / 2.0
        LJoyVert = self.axis_states.values()[4]
        LJoyHorz = self.axis_states.values()[5]
        DPadHorz = self.axis_states.values()[6]
        LBumper  = (self.axis_states.values()[7] + 1.0) / 2.0

        A = self.button_states.values()[0]
        B = self.button_states.values()[1]
        LJoyDown = self.button_states.values()[2]
        RTrigger = self.button_states.values()[3]
        Start = self.button_states.values()[4]
        LTrigger = self.button_states.values()[5]
        RJoyDown = self.button_states.values()[6]
        Mode = self.button_states.values()[7]
        Y   = self.button_states.values()[8]
        X   = self.button_states.values()[9]
        Select = self.button_states.values()[10]
        """ 
        0.RU   F     1.LU


        5.o          2.o


        4.RD        3.LD


        """

        #
        motors = [0, 0, 0, 0, 0, 0]

        #Let's first treat the X-Y plane as a vector, and get the unit vector
        #this is so that combined X and Y motions play nice
        vX = float(RJoyHorz)
        vY = float(RJoyVert)

        angle =  float((RBumper - LBumper) * 45.0)

        #if clockwise
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

        fmotors = [max(min(int(i*255), 255), -255) for i in motors]

        #Byte loses percision but means less data sending, so overall worth it.
        #To-do, ensure separately that this will work.
        print(fmotors)

        return


# Main event loop

def talker():
    #pub = rospy.Publisher('gamepad', gamepad, queue_size=10)
    #rospy.init_node('joystick_controller', anonymous=True)
    #rate = rospy.Rate(40) # 10hz
    joystick = Controller()
    while (joystick.get_button_states()[0] != 1):
        joystick.update()
        joystick.callback()
    joystick.jsdev.close()
 
if __name__ == '__main__':
    talker()

