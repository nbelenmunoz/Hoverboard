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

i2c = board.I2C()

sensor1 = adafruit_bno055.BNO055_I2C(i2c,0x28) #izquierdo
sensor2 = adafruit_bno055.BNO055_I2C(i2c,0x29) #derecho

izq = Motor(23,24,25)
der = Motor(17,27,22)

#PID weights and constants
Kp = 1
Ki = 0.1
Kd = 0.05

setpoint = 0
output_limits=(-50, 50)

limit = 1.5

# PID controllers for pitch and acceleration for both sensors
pid1_pitch = PID(Kp, Ki, Kd, setpoint, output_limits, auto_mode=True)
pid1_accel = PID(Kp, Ki, Kd, setpoint, output_limits, auto_mode=True)
pid2_pitch = PID(Kp, Ki, Kd, setpoint, output_limits, auto_mode=True)
pid2_accel = PID(Kp, Ki, Kd, setpoint, output_limits, auto_mode=True)

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
    
    # Calculate the control values for pitch and acceleration separately
    control1_pitch = pid1_pitch(quat1)
    control1_accel = pid1_accel(accel1)
    
    control2_pitch = pid2_pitch(quat2)
    control2_accel = pid2_accel(accel2)
    
    # Add the control values together to get the final control signal
    control1 = control1_pitch + control1_accel
    control2 = control2_pitch + control2_accel

    #Implementation of control
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
