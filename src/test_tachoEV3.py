from time import sleep, perf_counter_ns
import math
from gpiozero import DigitalInputDevice

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
    #print("tachoCount:", tachoCount)
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

startTime = perf_counter_ns()
prevTachoTime = perf_counter_ns()
prevPrintTime = perf_counter_ns()
prevTachoCount = 0
wheelAV = 0
loopCount = 0
while True:
    #calculate wheel angular velocity
    WHEEL_AV_INTERVAL = 100  #[ms]
    if (perf_counter_ns() - prevTachoTime) / 1e6 >= WHEEL_AV_INTERVAL:
        tachoTimeDelta = (perf_counter_ns() - prevTachoTime) / 1e9  #[sec]
        prevTachoTime = perf_counter_ns()
        
        pulses = tachoCount - prevTachoCount
        prevTachoCount = tachoCount
        cycles = pulses / 360   #360 pulses per rotation
        wheelAV = cycles / tachoTimeDelta * 2 * math.pi  #[rad/s]
    
    #debug print
    if (perf_counter_ns() - prevPrintTime) / 1e9 >= 1.0:
        secondsSinceLastPrint = (perf_counter_ns() - prevPrintTime) / 1e9
        prevPrintTime = perf_counter_ns()
        loopInterval = secondsSinceLastPrint / loopCount * 1000
        loopCount = 0
        print("tachoCount: %i, wheelAV: %.2f, loopInterval: %.2f ms"
              % (tachoCount, wheelAV, loopInterval))

    sleep(0.001)
    loopCount += 1

