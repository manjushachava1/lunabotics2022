import RPi.GPIO as GPIO
import time

Motor1A = 5
GPIO.setwarnings(False)

def setup():
    global pwm
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Motor1A,GPIO.OUT)
    GPIO.output(Motor1A, GPIO.LOW)
    pwm = GPIO.PWM(Motor1A, 255)
    pwm.start(0)
    
def loop():
    while True:
        for dc in range(0, 101, 1):
            pwm.ChangeDutyCycle(dc)
            time.sleep(0.01)
        for dc in range(100, 0, -1):
            pwm.ChangeDutyCycle(dc)
            time.sleep(0.01)
    
def destroy():
    pwm.stop()
    GPIO.output(Motor1A, GPIO.LOW)
    GPIO.cleanup()
    
if __name__ == '__main__':
    setup()
    try:
        loop()
    except KeyboardInterrupt:
        destroy()