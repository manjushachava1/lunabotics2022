from evdev import InputDevice, categorize, ecodes, KeyEvent
gamepad = InputDevice('/dev/input/event2')
from gpiozero import Servo
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory

factory = PiGPIOFactory(host='169.254.136.63')
servo_vert = Servo(25, pin_factory=factory)
servo_horiz = Servo(24, pin_factory=factory)
vert = 0
horiz = 0       

for event in gamepad.read_loop():
    if event.type == ecodes.EV_KEY:
        keyevent = categorize(event)
        if keyevent.keystate == KeyEvent.key_down:
            if keyevent.scancode == 289:
                servo_vert.value = vert
                sleep(0.5)
                vert = vert - 0.1
                print('back')
            if keyevent.scancode == 288:
                servo_horiz.value = horiz
                sleep(0.5)
                horiz = horiz - 0.1
                print('left')
            if keyevent.scancode == 291:
                servo_vert.value = vert
                sleep(0.5)
                vert = vert + 0.1
                print('forward')
            if keyevent.scancode == 290:
                servo_horiz.value = horiz
                sleep(0.5)
                horiz = horiz + 0.1
                print('right')
            if vert < -1:
                    vert = -1
            if vert > 1:
                    vert = 1
            print(vert)
            if horiz < -1:
                    vert = -1
            if horiz > 1:
                    horiz = 1
            print(horiz)

