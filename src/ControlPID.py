import time
import board
import adafruit_bno055
import numpy as np
import math
from gpiozero import Motor
from simple_pid import PID
import RPi.GPIO as GPIO

# GPIO initialized
GPIO.setmode(GPIO.BCM)

# GPIO Pins Motor 1 
enable_hover = 25
GPIO.setup(enable_hover, GPIO.OUT)

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

#PID weights and constants
Kp = 1
Ki = 0.1
Kd = 0.05

setpoint = 0
output_limits=(-50, 50)

limit = 1.5

# Controladores pid por motor. Los output limits son la velocidad de cada actuador
pid1 = PID(Kp, Ki, Kd, setpoint, output_limits, auto_mode=True)  
pid2 = PID(Kp, Ki, Kd, setpoint, output_limits, auto_mode=True) 

def pitch(quat, accel):
    if(quat[0] != None and quat[1] != None and quat[2] != None and quat[3] != None):
        pitch = np.arcsin(2 * quat[0] * quat[2] - quat[1] * quat[3])
    else:
        pitch = 0.0
    if math.isnan(pitch) or math.isnan(accel): 
        pitch = 0.0
        accel = 0.0
    return 57.2958 * pitch, accel

while True:
    quat1, accel1 = pitch(sensor1.quaternion, sensor1.acceleration[2])
    quat2, accel2 = pitch(sensor2.quaternion, sensor2.acceleration[2])
    
    # Calcula la salida dependiendo al pitch y la aceleracion
    control1 = pid1(quat1, accel1)
    control2 = pid2(quat2, accel2)

    #Implementacion del control 
    if control1 >= limit:
        izq.forward(1)
    elif control1 <= -limit:
        izq.backward(1)
    else:
        izq.stop()

    if control2 >= limit:
        der.forward(1)
    elif control2 <= -limit:
        der.backward(1)
    else:
        der.stop()

    print("Sensor 1")
    print("Pitch: {}".format( quat1))
    print("Acceleration: {}".format(accel1))
    print("Control: {}\n".format(control1))
    print("Sensor 2")
    print("Pitch: {}".format(quat2))
    print("Acceleration: {}".format(accel2))
    print("Control: {}\n".format(control2))

    time.sleep(0.05)
