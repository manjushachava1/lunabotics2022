from gpiozero import Button
import time
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import MCP3008


factory = PiGPIOFactory(host='192.168.0.4')
#exc = gpiozero.PWMOutputDevice(17,active_high=True,initial_value=0,frequency=255,pin_factory=factory)

# hall = MCP3008(channel=0)
hall = Button(2, pin_factory=factory)

print("Start")
while True:
#     print(hall.value)
    if hall.value == 1:
        print("end")
        time.sleep(1)