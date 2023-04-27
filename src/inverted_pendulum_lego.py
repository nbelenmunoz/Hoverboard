# type: ignore
#settings
ANGLE_FILTER_G    = 0.999 #gyro portion of complementary filter [0...1]
ANGLE_LIMIT       = 18    #stop program / start rise up sequence [deg]
RISEUP_END_ANGLE  = 5     #stop rise up sequence and start balancing [deg]
ANGLE_FIXRATE     = 1.0   #variate target angle [deg/s]
ANGLE_FIXRATE_2   = 0.1   #reduce continuous rotation
KP                = 0.2   #PID proportional factor
KI                = 0.5   #PID integral factor
KD                = 0.005 #PID derivative factor
MOTOR_R           = 13    #motor resistance [Ohm]
MOTOR_Ke          = 0.26  #motor back EMF constant [Vs/rad]
SUPPLY_VOLTAGE    = 8.1   #battery box voltage [V]
WHEEL_AV_INTERVAL = 100   #wheel angular velocity calculation interval [ms]
SLEEP_TIME        = 1     #main loop sleep [ms]

############################################################

from time import sleep, perf_counter_ns
from datetime import datetime
import math
import signal
from mpu9250_jmdev.registers import GFS_250, AFS_2G
from mpu9250_jmdev.mpu_9250 import MPU9250
from gpiozero import DigitalInputDevice, DigitalOutputDevice, PWMOutputDevice
from gpiozero.pins.pigpio import PiGPIOFactory

#prepare IMU sensor
imu = MPU9250(bus=1, gfs=GFS_250, afs=AFS_2G)
imu.abias = [0.013221153846153846, -0.030993339342948716, 0.004657451923076872]
imu.gbias = [1.3726094563802083, 1.2428309122721355, 0.00033772786458334]
imu.configure()

#prepare motor controller
my_factory = PiGPIOFactory()  #hardware pwm supported
motorPWM = PWMOutputDevice(12, pin_factory=my_factory)
motorDIR1 = DigitalOutputDevice(5)
motorDIR2 = DigitalOutputDevice(6)
motorPWM.frequency = 8000
motorPWM.value = 0
motorDIR1.value = 0
motorDIR2.value = 0

#prepare tachometers
tachoA = DigitalInputDevice(22)
tachoB = DigitalInputDevice(23)

tachoCount = 0
tachoAValue = tachoA.value
tachoBValue = tachoB.value
def tachoA_rising():
    global tachoCount
    global tachoAValue
    global tachoBValue
    tachoAValue = 1
    if tachoBValue == 0:
        #A in rising edge and B in low value
        #  => direction is clockwise (shaft end perspective)
        tachoCount += 1
    else:
        tachoCount -= 1
def tachoA_falling():
    global tachoAValue
    tachoAValue = 0
def tachoB_rising():
    global tachoCount
    global tachoAValue
    global tachoBValue
    tachoBValue = 1
    if tachoAValue == 0:
        tachoCount -= 1
    else:
        tachoCount += 1
def tachoB_falling():
    global tachoBValue
    tachoBValue = 0

tachoA.when_activated   = tachoA_rising
tachoA.when_deactivated = tachoA_falling
tachoB.when_activated   = tachoB_rising
tachoB.when_deactivated = tachoB_falling

#exit program when Ctrl-C is pressed
exitRequested = False
def sigintHandler(sig, frame):
    print("Ctrl-C pressed, exit program")
    global exitRequested
    exitRequested = True
signal.signal(signal.SIGINT, sigintHandler)
signal.signal(signal.SIGTERM, sigintHandler)

print("program started")
sleep(0.1)

startTime = perf_counter_ns()
prevLoopTime = perf_counter_ns()
prevTachoTime = perf_counter_ns()
prevPrintTime = perf_counter_ns()
loopCount = 0
gyroAngle = float('nan')
measuredAngle = float('nan')
risingUp = False
prevTachoCount = 0
wheelAV = 0
targetAngle = 0
error = 0
prevError = 0
integral = 0
derivative = 0
PIDoutput = 0
motorCtrl = 0
logData = [["secondsSinceStart","accAngle","gyroAngle","measuredAngle","targetAngle",
            "tachoCount","wheelAV","error","integral","derivative",
            "PIDoutput","motorCtrl"]]

while not exitRequested:
    timeDelta = (perf_counter_ns() - prevLoopTime) / 1e9  #[sec]
    prevLoopTime = perf_counter_ns()
    secondsSinceStart = (perf_counter_ns() - startTime) / 1e9
    
    #read accelerometer
    ax, ay, az = imu.readAccelerometerMaster()  #[G]
    accAngle = math.atan(-ax / math.sqrt(pow(ay, 2) + pow(az, 2))) * 180 / math.pi  #[deg]
    
    #read gyroscope
    gx, gy, gz = imu.readGyroscopeMaster()  #[deg/s]
    gyroAngleDelta = gz * timeDelta
    if math.isnan(gyroAngle): gyroAngle = accAngle
    gyroAngle += gyroAngleDelta  #[deg]
    
    #calculate arm angle (complementary filter)
    if math.isnan(measuredAngle): measuredAngle = accAngle
    measuredAngle = (ANGLE_FILTER_G * (measuredAngle + gyroAngleDelta) +
                     (1-ANGLE_FILTER_G) * accAngle)  #[deg]
    
    #safety check
    if abs(measuredAngle) >= ANGLE_LIMIT:
        if secondsSinceStart < 0.001:
            print("START RISE UP SEQUENCE")
            risingUp = True
        elif not risingUp:
            print("PROGRAM STOPPED, angle is too large: %.2f" % (measuredAngle))
            break
    
    #rise up sequence
    if risingUp:
        if secondsSinceStart < 1.0:
            #speed up reaction wheel to full speed
            motorPWM.value = 1.0
            motorDIR1.value = (measuredAngle < 0)
            motorDIR2.value = (measuredAngle > 0)
        elif secondsSinceStart < 1.5:
            #change direction using full power
            motorPWM.value = 1.0
            motorDIR1.value = (measuredAngle > 0)
            motorDIR2.value = (measuredAngle < 0)
            
            #wait until close to top position, then start balancing
            if abs(measuredAngle) < RISEUP_END_ANGLE:
                print("END RISE UP SEQUENCE")
                risingUp = False
        else:
            print("RISE UP TIMEOUT")
            break
    
    #calculate wheel angular velocity
    if (perf_counter_ns() - prevTachoTime) / 1e6 >= WHEEL_AV_INTERVAL:
        tachoTimeDelta = (perf_counter_ns() - prevTachoTime) / 1e9  #[sec]
        prevTachoTime = perf_counter_ns()
        
        pulses = tachoCount - prevTachoCount
        prevTachoCount = tachoCount
        cycles = pulses / 360   #360 pulses per rotation
        wheelAV = cycles / tachoTimeDelta * 2 * math.pi  #[rad/s]
    
    if not risingUp:
    
        #variate target angle
        if measuredAngle < targetAngle:
            targetAngle += ANGLE_FIXRATE * timeDelta
        else:
            targetAngle -= ANGLE_FIXRATE * timeDelta
        
        #reduce continuous rotation
        targetAngle -= ANGLE_FIXRATE_2 * wheelAV * timeDelta
        
        #PID controller
        error = targetAngle - measuredAngle
        integral += error * timeDelta
        derivative = (error - prevError) / timeDelta
        prevError = error
        PIDoutput = KP * error + KI * integral + KD * derivative
        
        #compensate for motor back EMF voltage
        current = -PIDoutput
        voltage = MOTOR_R * current + MOTOR_Ke * wheelAV
        
        #convert voltage to pwm duty cycle
        motorCtrl = voltage / SUPPLY_VOLTAGE
        
        #drive motor
        motorCtrl = min(max(motorCtrl, -1), 1)  #limit range to -1...1
        motorPWM.value = abs(motorCtrl)
        motorDIR1.value = (motorCtrl > 0)
        motorDIR2.value = (motorCtrl < 0)
    
    #log data for post analysis
    logData.append([secondsSinceStart, accAngle, gyroAngle, measuredAngle, targetAngle,
                    tachoCount, wheelAV, error, integral, derivative,
                    PIDoutput, motorCtrl])
    
    #debug print
    if (perf_counter_ns() - prevPrintTime) / 1e9 >= 1.0:
        secondsSinceLastPrint = (perf_counter_ns() - prevPrintTime) / 1e9
        prevPrintTime = perf_counter_ns()
        loopInterval = secondsSinceLastPrint / loopCount * 1000
        loopCount = 0
        print("measuredAngle: %.2f, motorCtrl: %.2f, loopInterval: %.2f ms"
              % (measuredAngle, motorCtrl, loopInterval))
    
    sleep(SLEEP_TIME / 1000)
    loopCount += 1

#stop motor
motorPWM.value = 0
motorDIR1.value = 0
motorDIR2.value = 0

#write log data to file
print("log size", len(logData), "rows")
if len(logData) > 0:
    filename = "datalog.dat"
    #filename = "datalog_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".dat"
    print("write to file:", filename)
    file = open(filename, "w")
    for logLine in logData:
        for value in logLine:
            file.write(str(value) + ' ')
        file.write('\n')
    file.close()

print("program ended")
