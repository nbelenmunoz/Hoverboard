from gpiozero import Motor
import adafruit_bno055
import board
import time
import math
import numpy as np


def pitch(quat, accel):
    if(quat[0] != None and quat[1] != None and quat[2] != None and quat[3] != None):
        pitch = np.arcsin(2 * quat[0] * quat[2] - quat[1] * quat[3])
    else:
        pitch = 0.0
    if math.isnan(pitch) or math.isnan(accel): 
        pitch = 0.0
        accel = 0.0
    return 57.2958 * pitch, accel

def main():
    # GPIO2 I2C1 SDA
    # GPIO3 I2C1 SCL

    i2c = board.I2C()

    sensor1 = adafruit_bno055.BNO055_I2C(i2c,0x28) #izquierdo
    sensor2 = adafruit_bno055.BNO055_I2C(i2c,0x29) #derecho

    # motor izquierdo GPIO pins 23(red) 24(black) 25(enable)
    # motor derecho GPIO pins 17(red) 27(black) 22(enable)

    izq = Motor(23,24,25)
    der = Motor(17,27,22)

    x1 = 0.0
    x2 = 0.0
    while(True):
        x1, acc1 = pitch(sensor1.quaternion, sensor1.acceleration[2])
        x2, acc2 =pitch(sensor2.quaternion, sensor2.acceleration[2])

        print("sensor 1: {}".format(x1))
        print("sensor 2: {}".format(x2))
    
    
        if(x1 != None and x2 != None):
            if(x1 < 1.5 and x1 > 358.5):
                izq.stop()

            if(x2 < 1.5 and x2 > 358.5):
                der.stop()

            if(x1 > 1.5 and x1 < 50):
                izq.forward(1)
            
            if(x1 < 358.5 and x1 > 350):
                izq.backward(1)

            if(x2 > 1.5 and x2 < 50):
                der.forward(1)
            
            if(x2 < 358.5 and x2 > 350):
                der.backward(1)

        time.sleep(0.01)

main()
