from evdev import InputDevice, categorize, ecodes, KeyEvent
gamepad = InputDevice('/dev/input/event2')
import gpiozero
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory

factory = PiGPIOFactory(host='169.254.136.63')
m1 = gpiozero.PWMOutputDevice(5,active_high=True,initial_value=0,frequency=255,pin_factory=factory)
speed = .36    

for event in gamepad.read_loop():
    if event.type == ecodes.EV_KEY:
        keyevent = categorize(event)
        if keyevent.keystate == KeyEvent.key_down:
            if keyevent.scancode == 289:
                speed = speed - .01
                m1.value = speed
                sleep(0.1)
                print(speed)
            if keyevent.scancode == 291:
                speed = speed + .01
                m1.value = speed
                sleep(0.1)
                print(speed)
            if keyevent.scancode == 296:
                m1.value = 0
                sleep(0.1)
                print('end')
            
