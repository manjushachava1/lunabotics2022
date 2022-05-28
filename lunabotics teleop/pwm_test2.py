import RPi.GPIO as GPIO
from time import sleep
from evdev import InputDevice, categorize, ecodes, KeyEvent

gamepad = InputDevice('/dev/input/event2')
GPIO.setwarnings(False)
Motor1A = 5
speed = 36.5

GPIO.setmode(GPIO.BCM)
GPIO.setup(Motor1A,GPIO.OUT)
GPIO.output(Motor1A, GPIO.LOW)
p = GPIO.PWM(Motor1A, 255)
p.start(0)

for event in gamepad.read_loop():
    if event.type == ecodes.EV_KEY:
        keyevent = categorize(event)
        if keyevent.keystate == KeyEvent.key_down:
            if keyevent.scancode == 289:
                speed = speed - .1
                p.ChangeDutyCycle(speed)
                sleep(0.1)
                print('back')
            if keyevent.scancode == 291:
                speed = speed + .1
                p.ChangeDutyCycle(speed)
                sleep(0.1)
                print('forward')
                
            print(speed)