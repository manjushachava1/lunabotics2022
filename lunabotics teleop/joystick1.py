from evdev import InputDevice, categorize, ecodes, KeyEvent
import gpiozero
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory

gamepad = InputDevice('/dev/input/event2')
last = {"ABS_RZ": 128, "ABS_Y": 128}
factory = PiGPIOFactory(host='192.168.0.4')
m1 = gpiozero.PWMOutputDevice(13,active_high=True,initial_value=0,frequency=255,pin_factory=factory)
m2 = gpiozero.PWMOutputDevice(6,active_high=True,initial_value=0,frequency=255,pin_factory=factory)
speedR = .36
speedL = .36
transCO = .175

for event in gamepad.read_loop():
    if event.type == ecodes.EV_ABS:
        absevent = categorize(event)
        if ecodes.bytype[absevent.event.type][absevent.event.code] =='ABS_RZ':
            last["ABS_RZ"] = absevent.event.value
            
            speedR = ((last["ABS_RZ"]*transCO)+14)/100
            m1.value = speedR
            sleep(0.01)
            print (last["ABS_RZ"])
            print(speedR)
            
        if last["ABS_RZ"] == 127:
                m1.value = 0
                sleep(0.01)
                
        if ecodes.bytype[absevent.event.type][absevent.event.code] =='ABS_Y':
            last["ABS_Y"] = absevent.event.value
            
            speedL = (((255-last["ABS_Y"])*transCO)+14)/100
            m2.value = speedL
            sleep(0.01)
            print (255-last["ABS_Y"])
            print(speedL)
            
        if last["ABS_Y"] == 127:
                m2.value = 0
                sleep(0.01)