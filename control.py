import time
import board
import adafruit_bno055
import numpy as np
import math
from gpiozero import Motor
from simple_pid import PID
import RPi.GPIO as GPIO
#import matplotlib as mp

#GPIO inicializafo
GPIO.setmode(GPIO.BCM)
GPIO.setup(16,GPIO.OUT)

limit = 2.0
accelLimit = 0.3

#ganancias
kp1 = 1
ki1 = 0.1
kd1 = 1

kp2 = 1
ki2 = 0.5
kd2 = 1.5

i2c = board.I2C()  # uses board.SCL and board.SDA
sensor1 = adafruit_bno055.BNO055_I2C(i2c,0x28)
sensor2 = adafruit_bno055.BNO055_I2C(i2c,0x29)

# motor izquierdo GPIO pins 23(red) 24(black) 25(enable)
# motor derecho GPIO pins 17(red) 27(black) 22(enable)

izq = Motor(23,24,25)
der = Motor(17,27,22)

# Controladores pid por motor. Los output limits son la velocidad de cada actuador
pidangle1 = PID(kp1, ki1, kd1, setpoint=0, output_limits=(-15, 15))
pidaccel1 = PID(kp2, ki2, kd2, setpoint=0, output_limits=(-5, 5))
pidangle2 = PID(kp1, ki1, kd1, setpoint=0, output_limits=(-15, 15))
pidaccel2 = PID(kp2, ki2, kd2, setpoint=0, output_limits=(-5, 5))

def pitch(quat):
    # Create Pitch Angle from Quaternions
    if(quat[0] != None and quat[1] != None and quat[2] != None and quat[3] != None):
        pitch = np.arcsin(2 * quat[0] * quat[2] - quat[1] * quat[3])
    else:
        pitch = 0.0
    if(math.isnan(pitch)):
        pitch = 0.0
# Convert Radians to Degrees */
    return  57.2958 * pitch

def accel(acc):
    #print(type(acc))
    if type(acc) != 'float':
        acc = 0.0
    return acc

def test():
    while(true):
        GPIO.output(16,GPIO.HIGH)

def start():
    on_middle = False
    i = 0
    while(not on_middle):
        i += 1
        izq.forward(1)
        der.forward(1)
        if (i == 100):
            on_middle = True

def main():
    while(True):
        GPIO.output(16,GPIO.HIGH)
        #if(sensor1.calibrated and sensor2.calibrated):
        x1 = pitch(sensor1.quaternion)
        accel1 = sensor1.acceleration[0]
        #acc = sensor1.acceleration

        x2 = pitch(sensor2.quaternion)
        accel2 = sensor2.acceleration[0]

        #print(acc)

        print("Sensor 1")
        print("angle: {}".format(x1))
        print("accel: {}".format(accel1))
        print("Sensor 2")
        print("angl:e: {}".format(x2))
        print("accel: {}".format(accel2))
        #time.sleep(0.1)

        #if(x1 < limit and x1 > -limit):
        #    izq.stop()
        #if(x1 > limit):
        #    izq.backward(1)
        #if(x1 < -limit):
        #    izq.forward(1)
        #if(x2 < limit and x2 > -limit):
        #    der.stop()
        #if(x2 > -limit):
        #    der.backward(1)
        #if(x2 < limit):
        #    der.forward(1)
        # Calcula la salida del control
       
        controlangle1 = pidangle1(x1)
        controlaccel1 = pidaccel1(accel1)

        controlangle2 = pidangle2(x2)
        controlaccel2 = pidaccel2(accel2)
 
        # Aplicaci[on del contol
        if controlangle1 > limit or controlaccel1 > limit:
            izq.forward(1)
        elif controlangle1 < -limit or controlaccel1 < -limit:
            izq.backward(1)
        else:
            izq.stop()

        if controlangle2 > limit or controlaccel2 > accelLimit:
            der.forward(1)
        elif controlangle2 < -limit or controlaccel2 < -accelLimit:
            der.backward(1)
        else: 
            der.stop()

        time.sleep(0.1)
main()



