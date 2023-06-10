from gpiozero import Motor
import adafruit_bno055
import board
import time
import serial

uart = serial.Serial("/dev/serial1")
sensor = adafruit_bno055.BNO055_UART(uart) #izquierdo

# motor izquierdo GPIO pins 23(red) 24(black) 25(enable)

izq = Motor(23,24,25)

x = 0.0
while(True):
    print(sensor.euler[0])
    x = sensor.euler[0]
    if(x != None):
        if(x < 1.5 and x > 358.5):
            izq.stop()

        if(x > 1.5 and x < 50):
            izq.forward(1)

        if(x < 358.5 and x > 350):
            izq.backward(1)
