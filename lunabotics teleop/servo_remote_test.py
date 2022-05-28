from gpiozero import Servo
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory

factory = PiGPIOFactory(host='192.168.43.198')
servo = Servo(25, pin_factory=factory)

try:
    while True:
        servo.min()
        sleep(2.5)
        servo.max()
        sleep(2.5)
except KeyboardInterrupt:
    print("Program stopped")