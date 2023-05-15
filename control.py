from gpiozero import Motor
import adafruit_bno055
import board
import time
import numpy as np

def pitch(quat):
    #create pitch angle from quaternion
    pitch = np.arcsin(2*quat.w()*quat.y()-quat.x()*quat.z())

    #convert Radians to degrees
    return 57.2958 * pitch

def main():
    # GPIO2 I2C1 SDA
    # GPIO3 I2C1 SCL

    i2c = board.I2C()

    sensor1 = adafruit_bno055.BNO055_I2C(i2c) #izquierdo
    sensor2 = adafruit_bno055.BNO055_I2C(i2c) #derecho

    # motor izquierdo GPIO pins 23(red) 24(black) 25(enable)
    # motor derecho GPIO pins 17(red) 27(black) 22(enable)

    izq = Motor(23,24,25)
    der = Motor(17,27,22)

    x1 = 0.0
    x2 = 0.0
    while(True):
        print("sensor 1:")
        print(sensor1.euler[0])

        print("sensor 2: ")
        print(sensor2.euler[0])
    
        x1 = sensor1.euler[0]
        x2 = sensor2.euler[0]
    
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
