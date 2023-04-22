from time import sleep
from gpiozero import DigitalOutputDevice, PWMOutputDevice
from gpiozero.pins.pigpio import PiGPIOFactory

my_factory = PiGPIOFactory()  #hardware pwm supported
motorPWM = PWMOutputDevice(12, pin_factory=my_factory)
motorDIR1 = DigitalOutputDevice(5)
motorDIR2 = DigitalOutputDevice(6)

motorPWM.frequency = 8000

#TB6612FNG direction specs:
#  Dir1=0 Dir2=0: Stop (high impedance)
#  Dir1=1 Dir2=0: CW
#  Dir1=0 Dir2=1: CCW
#  Dir1=1 Dir2=1: Short brake

print("Drive motor 50% CW")
motorDIR1.value = 1
motorDIR2.value = 0
motorPWM.value = 0.5
sleep(1)

print("Stop motor")
motorDIR1.value = 0
motorDIR2.value = 0
motorPWM.value = 0.0
sleep(1)

print("Drive motor 100% CCW")
motorDIR1.value = 0
motorDIR2.value = 1
motorPWM.value = 1.0
sleep(1)

print("Stop motor using short brake")
motorDIR1.value = 1
motorDIR2.value = 1
motorPWM.value = 0.0
sleep(2)

print("Drive motor 0-100% CCW")
motorDIR1.value = 0
motorDIR2.value = 1
motorPWM.value = 0
for i in range(101):
    motorPWM.value = i / 100
    print("  pwm %i %%" % (motorPWM.value * 100))
    sleep(0.02)
sleep(1)
for i in range(101):
    motorPWM.value = 1.0 - i / 100
    print("  pwm %i %%" % (motorPWM.value * 100))
    sleep(0.02)

print("Stop motor")
motorDIR1.value = 0
motorDIR2.value = 0
motorPWM.value = 0.0
