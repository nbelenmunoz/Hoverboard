from time import sleep, perf_counter_ns
import math
from mpu9250_jmdev.registers import GFS_250, AFS_2G
from mpu9250_jmdev.mpu_9250 import MPU9250

imu = MPU9250(bus=1, gfs=GFS_250, afs=AFS_2G)
imu.abias = [0.013221153846153846, -0.030993339342948716, 0.004657451923076872]
imu.gbias = [1.3726094563802083, 1.2428309122721355, 0.10033772786458334]
imu.configure()

#imu.calibrateMPU6500()
#print("imu.abias: ",  imu.abias)
#print("imu.gbias: ",  imu.gbias)

startTime = perf_counter_ns()
lastGyroTime = perf_counter_ns()
lastPrintTime = perf_counter_ns()
time = 0
loopCount = 0
gyroAngle = float('nan')
filteredAngle = float('nan')
while time < 10:
    #read accelerometer
    ax, ay, az = imu.readAccelerometerMaster()  #[G]
    accAngle = math.atan(-ax / math.sqrt(pow(ay, 2) + pow(az, 2))) * 180 / math.pi  #[deg]
    
    #read gyroscope
    gx, gy, gz = imu.readGyroscopeMaster()  #[deg/s]
    timeDelta = (perf_counter_ns() - lastGyroTime) / 1e9  #[sec]
    lastGyroTime = perf_counter_ns()
    gyroAngleDelta = gz * timeDelta
    if math.isnan(gyroAngle): gyroAngle = accAngle
    gyroAngle += gyroAngleDelta  #[deg]
    
    #complementary filter
    if math.isnan(filteredAngle): filteredAngle = accAngle
    filteredAngle = 0.999 * (filteredAngle + gyroAngleDelta) + 0.001 * accAngle
    
    #debug print
    if (perf_counter_ns() - lastPrintTime) / 1e9 >= 1.0:
        secondsSincePrint = (perf_counter_ns() - lastPrintTime) / 1e9
        lastPrintTime = perf_counter_ns()
        loopInterval = secondsSincePrint / loopCount * 1000
        loopCount = 0
        print("accAngle %.2f, gyroAngle %.2f, filteredAngle %.2f, loopInterval %.2f ms"
           % (accAngle, gyroAngle, filteredAngle, loopInterval))
    
    sleep(0.001)
    loopCount += 1
