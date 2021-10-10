import pygame
import time
import sys

class Controller:
    def __init__(self):
        pygame.joystick.init()
        if not pygame.joystick.get_count():
            print("Controller not detected")
            sys.exit()

        self.joystick = pygame.joystick.Joystick(0)

    def attributes(self):
        print("Name:", self.joystick.get_name())
        print("Number of Axis:", self.joystick.get_numaxes())
        print("Number of Buttons:", self.joystick.get_numbuttons())

    def get_left_joystick(self, precision=2):
        return (round(self.joystick.get_axis(0), precision), round(self.joystick.get_axis(1)*-1, precision))

    def left_joystick_log(self, precision=2):
        coordinates = self.get_left_joystick(precision)
        print(f"Left Joy - X: {coordinates[0]}, Y: {coordinates[1]}")

    def get_buttons(self):
        return tuple(self.joystick.get_button(i) for i in range(4))

    def buttons_log(self):
        buttons = self.get_buttons()
        print(f"Buttons - A: {buttons[0]}, B: {buttons[1]}, X: {buttons[2]}, Y: {buttons[3]}")

    def get_dpad(self):
        return self.joystick.get_hat(0)

    def dpad_log(self):
        dpad = self.get_dpad()
        print(f"Dpad - X: {dpad[0]}, Y: {dpad[1]}")

