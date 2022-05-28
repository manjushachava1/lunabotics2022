from evdev import InputDevice, categorize, ecodes, KeyEvent
import gpiozero
from gpiozero import Servo
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Button

gamepad = InputDevice('/dev/input/event2')
factory = PiGPIOFactory(host='192.168.0.4')
home_factory = PiGPIOFactory(host='192.168.0.10')
last = {"ABS_RZ": 128, "ABS_Y": 128}
transCO = .175
hall = Button(2, pin_factory=factory)
estop = Button(2)

# motors
exc = gpiozero.PWMOutputDevice(17,active_high=True,initial_value=0,frequency=255,pin_factory=factory)
exc_speed = .37
sto = gpiozero.PWMOutputDevice(23,active_high=True,initial_value=0,frequency=255,pin_factory=factory)
sto_speed = .37
step = gpiozero.PWMOutputDevice(22,active_high=True,initial_value=0,frequency=255,pin_factory=factory)
step_speed = .37
piv = gpiozero.PWMOutputDevice(27,active_high=True,initial_value=0,frequency=255,pin_factory=factory)
piv_speed = .37
m1 = gpiozero.PWMOutputDevice(5,active_high=True,initial_value=0,frequency=255,pin_factory=factory)
speedR = .37
m2 = gpiozero.PWMOutputDevice(6,active_high=True,initial_value=0,frequency=255,pin_factory=factory)
speedL = .37


# and not hall.is_pressed

print('start')
for event in gamepad.read_loop():
        if hall.is_pressed:
            step_speed = .36
            step.value = step_speed
            print("end")
            sleep(1)
        if event.type == ecodes.EV_ABS:
            absevent = categorize(event)
#drive motors
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

        
#buttons
        if event.type == ecodes.EV_KEY:
            keyevent = categorize(event)
            if keyevent.keystate == KeyEvent.key_down:
                if keyevent.scancode == 293:
                    exc_speed = exc_speed - .05
                    exc.value = exc_speed
                    sleep(0.1)
                    print('EXC:', '{0:.4f}'.format(exc_speed), 'reverse\n')
                if keyevent.scancode == 295:
                    exc_speed = exc_speed + .05
                    exc.value = exc_speed
                    sleep(0.1)
                    print('EXC:', '{0:.4f}'.format(exc_speed), 'dig\n')
                if keyevent.scancode == 294:
                    sto_speed = sto_speed - .05
                    sto.value = sto_speed
                    sleep(0.1)
                    print('STO:', '{0:.4f}'.format(sto_speed), 'dump\n')
                if keyevent.scancode == 292:
                    sto_speed = sto_speed + .05
                    sto.value = sto_speed
                    sleep(0.1)
                    print('STO:', '{0:.4f}'.format(sto_speed), 'reverse\n')
                if keyevent.scancode == 288:
                    #speed down
                    step_speed = .16
                    step.value = step_speed
                    sleep(0.1)
                    print('STEP:', '{0:.4f}'.format(step_speed), 'down\n')
                if keyevent.scancode == 290:
                    #full speed
                    step_speed = .55
                    step.value = step_speed
                    sleep(0.1)
                    print('STEP:', '{0:.4f}'.format(step_speed), 'up\n')
                if keyevent.scancode == 297:
                    #stopped lead screw
                    step_speed = .0
                    step.value = step_speed
                    sleep(0.1)
                    print('STEP:', '{0:.4f}'.format(step_speed), 'stop\n')
                if keyevent.scancode == 289:
                    #updated speed
                    piv_speed = piv_speed - .01
                    piv.value = piv_speed
                    sleep(0.1)
                    print('PIV:', '{0:.4f}'.format(piv_speed), 'down\n')
                if keyevent.scancode == 291:
                    #update speed
                    piv_speed = piv_speed + .01
                    piv.value = piv_speed
                    sleep(0.1)
                    print('PIV:', '{0:.4f}'.format(piv_speed), 'up\n')
                if keyevent.scancode == 296:
                    #stopped piv
                    piv_speed = .37
                    piv.value = piv_speed
                    sleep(0.1)
                    print('PIV:', '{0:.4f}'.format(piv_speed), 'stop\n')
            
