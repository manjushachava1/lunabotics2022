from evdev import InputDevice, categorize, ecodes
gamepad = InputDevice('/dev/input/event2')
from time import sleep

for event in gamepad.read_loop():
    if event:
            print(event)