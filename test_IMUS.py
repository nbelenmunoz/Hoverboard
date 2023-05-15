import time
import board
import adafruit_bno055


i2c = board.I2C()  # uses board.SCL and board.SDA
sensor1 = adafruit_bno055.BNO055_I2C(i2c,0x28)
sensor2 = adafruit_bno055.BNO055_I2C(i2c,0x29)

last_val = 0xFFFF

def pitch(quaternion):
    # Create Pitch Angle from Quaternions
    pitch = np.arcsin(2 * quat.w() * quat.y() - quat.x() * quat.z())

    # Convert Radians to Degrees */
    return  57.2958 * pitch

while True:
    quat1 = pitch(sensor1.quaternion)
    quat2 = pitch(sensor2.quaternion)
    print(quat1)
    print(quat2)
    time.sleep(0.01)

