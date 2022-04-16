#!/usr/bin/env python
# https://gist.github.com/rdb/8864666
#  Released by rdb under the Unlicense (unlicense.org)
# Based on information from:
# https://www.kernel.org/doc/Documentation/input/joystick-api.txt

import os, struct, array, time
from fcntl import ioctl

# These constants were borrowed from linux/input.h

import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
#from sensor_msgs.msg import Joy

from rov_control.msg import gamepad
msg = gamepad()


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
                print(axis)
                #if axis:
                fvalue = value / 32767.0
                self.axis_states[axis] = fvalue
            print(self.axis_states)
            print(self.button_states)
    def get_axis_states(self):
        return self.axis_states.values()
    def get_button_states(self):
        return self.button_states.values()

# Main event loop

def talker():
    pub = rospy.Publisher('gamepad', gamepad, queue_size=10)
    rospy.init_node('joystick_controller', anonymous=True)
    rate = rospy.Rate(40) # 10hz
    joystick = Controller()
    while not rospy.is_shutdown():
        joystick.update()
        msg = gamepad()
        msg.Axis = joystick.get_axis_states()
        msg.Buttons = joystick.get_button_states()
        pub.publish(msg)
        rate.sleep()
    joystick.jsdev.close()
 
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

