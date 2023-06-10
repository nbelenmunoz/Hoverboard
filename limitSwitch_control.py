import time
import board
import adafruit_bno055
import numpy as np
import math
from gpiozero import Motor
from simple_pid import PID
import RPi.GPIO as GPIO
#import matplotlib.pyplot as plt

# set GPIO pins as constants
hover = 16
izqBackLimit = 6
izqFrontLimit = 13
derBackLimit = 19
derFrontLimit = 26

#GPIO inicializado
GPIO.setmode(GPIO.BCM)
GPIO.setup(16,GPIO.OUT)

GPIO.setup(izqBackLimit,GPIO.IN, pull_up_down=GPIO.PUD_UP) # pin GPIO 6 como entrada 
GPIO.setup(izqFrontLimit,GPIO.IN, pull_up_down=GPIO.PUD_UP) # pin GPIO 13 como entrada
GPIO.setup(derBackLimit,GPIO.IN, pull_up_down=GPIO.PUD_UP) # pin GPIO 19 como entrada
GPIO.setup(derFrontLimit,GPIO.IN, pull_up_down=GPIO.PUD_UP) # pin GPIO 26 como entrada

#GPIO.output(16,GPIO.LOW)

limit = 2.0
accelLimit = 0.3

#ganancias

kp1 = 1
ki1 = 0
kd1 = 0.5

kp2 = 2
ki2 = 0
kd2 = 0.5

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
    if None in quat:
        return 0.0

    sin_value = 2 * quat[0] * quat[2] - quat[1] * quat[3]
    if sin_value > 1:
        sin_value = 1
    elif sin_value < -1:
        sin_value = -1

    pitch = np.arcsin(sin_value)
    return 57.2958 * pitch

def accel(acc):
    if not isinstance(acc, float):
        acc = 0.0
    elif acc is None:
        return 0.0
    return acc


#def test():
#    while(true):
#        GPIO.output(16,GPIO.HIGH)

def start():
    on_middle = False
    i = 0
    while(not on_middle):
        i += 1
        izq.forward(1)
        der.forward(1)
        if (i == 100):
            on_middle = True


def turn_on():
    GPIO.output(16,GPIO.HIGH)

def initial_routine():
    izq.backward(1)  # move forward at (0 <= x <= 1) [float]
    der.backward(1)
    time.sleep(3)
    izq.forward(1)
    der.forward(1)
    time.sleep(1.15)
    der.stop()
    izq.stop()
#    turn_on()
def main():

    while(True):

        #if(sensor1.calibrated and sensor2.calibrated):
        x1 = pitch(sensor1.quaternion)
        accel1 = sensor1.acceleration[0] if sensor1.acceleration[0] is not None else 0.0
        #acc = sensor1.acceleration

        x2 = pitch(sensor2.quaternion)
        accel2 = sensor2.acceleration[0] if sensor2.acceleration[0] is not None else 0.0

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

        time_values = []
        pid_output_values = []

        # Aplicacion del contol
        if controlangle1 > limit or controlaccel1 > limit and not GPIO.input(izqBackLimit):
            izq.forward(1)
        elif controlangle1 < -limit or controlaccel1 < -limit and not GPIO.input(izqFrontLimit):
            izq.backward(1)
        else:
            izq.stop()

        if controlangle2 > limit or controlaccel2 > accelLimit and not GPIO.input(derBackLimit):
            der.forward(1)
        elif controlangle2 < -limit or controlaccel2 < -accelLimit and not GPIO.input(derFrontLimit):
            der.backward(1)
        else: 
            der.stop()


        # Plot the PID output over time
#        plt.plot(time_values, pid_output_values)
#        plt.xlabel('Time')
#        plt.ylabel('PID Output')
#        plt.title('PID Controller Output')
#        plt.grid(True)
#        plt.show()
#        
        time.sleep(0.1)

initial_routine()
#time.sleep(5)
turn_on()
main()