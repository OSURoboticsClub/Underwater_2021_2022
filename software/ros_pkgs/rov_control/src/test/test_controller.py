import pygame
import sys
sys.path.append("../")
from controller import Controller

DELAY_TIME = 1000 # ms

if __name__ == "__main__":
    pygame.init()
    controller = Controller()
    while True:
        pygame.event.pump()
        controller.left_joystick_log()
        controller.buttons_log()
        controller.dpad_log()
        print()
        pygame.time.delay(DELAY_TIME)
