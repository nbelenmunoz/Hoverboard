import time
import board
import adafruit_bno055
import numpy as np
import math
from simple_pid import PID
import RPi.GPIO as GPIO

#GPIO inicializafo
GPIO.setmode(GPIO.BCM)

#GPIO Pins Motor 1 
motor1_in1 = 24
motor1_in2 = 23
motor1_en = 25
GPIO.setup(motor1_in1,GPIO.OUT)
GPIO.setup(motor1_in2,GPIO.OUT)
GPIO.setup(motor1_en,GPIO.OUT)
motor1_pwm = GPIO.PWM(motor1_en,100)
motor1_pwm.start(0)

#GPIO Pins Motor 2 
motor2_in1 = 27
motor2_in2 = 22
motor2_en = 17
GPIO.setup(motor2_in1,GPIO.OUT)
GPIO.setup(motor2_in2,GPIO.OUT)
GPIO.setup(motor2_en,GPIO.OUT)
motor2_pwm = GPIO.PWM(motor2_en,100)
motor2_pwm.start(0)

i2c = board.I2C()  
sensor1 = adafruit_bno055.BNO055_I2C(i2c,0x28)
sensor2 = adafruit_bno055.BNO055_I2C(i2c,0x29)

# Controladores pid por motor. Los output limits son la velocidad de cada actuador
pid1 = PID(1, 0.1, 0.05, setpoint=0, output_limits=(-50, 50))  
pid2 = PID(1, 0.1, 0.05, setpoint=0, output_limits=(-50, 50)) 

def pitch(quat):
    if(quat[0] != None and quat[1] != None and quat[2] != None and quat[3] != None):
        pitch = np.arcsin(2 * quat[0] * quat[2] - quat[1] * quat[3])
    else:
        pitch = 0.0
    if(math.isnan(pitch)):
        pitch = 0.0
    return  57.2958 * pitch
  
while True:
    quat1 = pitch(sensor1.quaternion)
    quat2 = pitch(sensor2.quaternion)
    
    # Calcula la salida del control
    control1 = pid1(quat1)
    control2 = pid2(quat2)

    # Aplicaci[on del contol
    if control1 >= 0:
        GPIO.output(motor1_in1,GPIO.HIGH)
        GPIO.output(motor1_in2,GPIO.LOW)
        motor1_pwm.ChangeDutyCycle(min(abs(control1), 100))
    else:
        GPIO.output(motor1_in1,GPIO.LOW)
        GPIO.output(motor1_in2,GPIO.HIGH)
        motor1_pwm.ChangeDutyCycle(min(abs(control1), 100))

    if control2 >= 0:
        GPIO.output(motor2_in1,GPIO.HIGH)
        GPIO.output(motor2_in2,GPIO.LOW)
        motor2_pwm.ChangeDutyCycle(min(abs(control2), 100))
    else:
        GPIO.output(motor2_in1,GPIO.LOW)
        GPIO.output(motor2_in2,GPIO.HIGH)
        motor2_pwm.ChangeDutyCycle(min(abs(control2), 100))

    print("Sensor 1")
    print("Pitch: ", quat1)
    print("Control: ", control1)
    print("Sensor 2")
    print("Pitch: ", quat2)
    print("Control: ", control2)

    time.sleep(0.1)
