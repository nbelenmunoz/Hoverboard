import time
import board
import busio
import adafruit_bno055
import RPi.GPIO as GPIO
from simple_pid import PID

# Setup BNO055 sensor
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

# Setup PID controller
pid = PID(10, 5, 2, setpoint=0)  # Adjust PID parameters as necessary
pid.output_limits = (-50, 50)  # Adjust limits according to your actuator

# Setup Motor Controller (H-bridge)
in1 = 5  # GPIO pin numbers, adjust as necessary
in2 = 6
enA = 9

GPIO.setmode(GPIO.BCM)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(enA, GPIO.OUT)
motor = GPIO.PWM(enA, 1000)  # 1000 Hz frequency
motor.start(0)

def control_motor(speed):
    if speed > 0:
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
    else:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
        speed = -speed
    motor.ChangeDutyCycle(speed)

try:
    while True:
        angle = sensor.euler[2]  # Z orientation
        output = pid(angle)
        control_motor(output)
        time.sleep(0.01)  # Adjust as necessary

except KeyboardInterrupt:
    print("Exiting...")

finally:
    motor.stop()
    GPIO.cleanup()
