import time
import board
import adafruit_bno055
import numpy as np
import math

i2c = board.I2C()  # uses board.SCL and board.SDA
sensor1 = adafruit_bno055.BNO055_I2C(i2c,0x28)
sensor2 = adafruit_bno055.BNO055_I2C(i2c,0x29)

last_val = 0xFFFF

def pitch(quat):
    # Create Pitch Angle from Quaternions
    #print(quat[0])
    #print(quat[1])
    #print(quat[2])
    #print(quat[3])
    if(quat[0] != None and quat[1] != None and quat[2] != None and quat[3] != None):
        pitch = np.arcsin(2 * quat[0] * quat[2] - quat[1] * quat[3])
    else:
        pitch = 0.0
    if(math.isnan(pitch)):
        pitch = 0.0
    # Convert Radians to Degrees */
    return  57.2958 * pitch

while True:
    #if(sensor1.calibrated and sensor2.calibrated):
    quat1 = pitch(sensor1.quaternion)
    quat2 = pitch(sensor2.quaternion)
    print("Sensor 1")
    print(quat1)
    print("Sensor 2")
    print(quat2)
    time.sleep(0.1)
        
