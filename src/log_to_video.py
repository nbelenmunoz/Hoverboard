# type: ignore

from time import sleep, perf_counter_ns
from datetime import datetime
import math
import os
import matplotlib.pyplot as pyplot
from matplotlib.animation import FFMpegFileWriter

#datalog.dat format:
#  logData = [["secondsSinceStart", "accAngle", "gyroAngle", "measuredAngle", "targetAngle",
#            "tachoCount", "wheelAV", "error", "integral", "derivative", "PIDoutput", "motorCtrl"]]

FILENAME = "datalog.dat"
SELECTED = [1,"accelerometer angle [deg]",2,"gyroscope angle [deg]",3,"filtered angle [deg]"]
XY_AREA = [-5, 0, -20, 20]
TARGET_FPS = 25
FIGSIZE = (8,8)
LEGEND_POS = [0.0, 1.30, 'upper left', 16]
pyplot.rcParams['axes.prop_cycle'] = pyplot.cycler('color', ['#1f77b4', '#ff7f0e', '#8c564b', '#2ca02c', '#7f7f7f', '#bcbd22', '#17becf']) 

##########################################################################

X_MIN = XY_AREA[0]
X_MAX = XY_AREA[1]
Y_MIN = XY_AREA[2]
Y_MAX = XY_AREA[3]
SELECTED_DATA = []
LEGEND_TITLES = []
for s in SELECTED:
    if type(s) == int: SELECTED_DATA.append(s)
    else: LEGEND_TITLES.append(s)

print("read from file")
file = open(FILENAME, "r")
fileLines = file.readlines()
file.close()

print("parse data")
logData = []
for fileLine in fileLines:
    va = fileLine.strip().split(' ')
    if fileLine != fileLines[0]:
        va = [float(v) for v in va]
        secondsSinceStart = va[0]
    va2 = [va[0]]
    for i in SELECTED_DATA:
        va2.append(va[i])
    logData.append(va2)

#create tables for real time data
if len(LEGEND_TITLES) == 0:
    LEGEND_TITLES = logData[0][1:]
xValues = []
yValuesArray = []
for i in range(1,len(logData[0])):
    yValuesArray.append([])

#find y axis range
if math.isnan(Y_MIN):
    for logLine in logData[1:]:
        for i in range(1,len(logLine)):
            if math.isnan(Y_MIN) or logLine[i] < Y_MIN: Y_MIN = logLine[i]
    Y_MIN = math.floor(Y_MIN)
if math.isnan(Y_MAX):
    for logLine in logData[1:]:
        for i in range(1,len(logLine)):
            if math.isnan(Y_MAX) or logLine[i] > Y_MAX: Y_MAX = logLine[i]
    Y_MAX = math.ceil(Y_MAX)

#create graph
print("create graph")
pyplot.rcParams.update({'font.size':13})
fig = pyplot.figure(figsize=FIGSIZE)
pyplot.grid()
pyplot.axis([X_MIN, X_MAX, Y_MIN, Y_MAX])
pyplot.xlabel('time [sec]', fontsize=16)
plots = []
for yValues in yValuesArray:
    plot = pyplot.plot(xValues, yValues, linewidth=2.5)[0]
    plots.append(plot)
if len(LEGEND_POS):
    pyplot.legend(LEGEND_TITLES, bbox_to_anchor=(LEGEND_POS[0], LEGEND_POS[1]), loc=LEGEND_POS[2], fontsize=LEGEND_POS[3])
else:
    pyplot.legend(LEGEND_TITLES, loc='upper left', fontsize=16)
pyplot.tight_layout()
pyplot.draw()
pyplot.ioff()
pyplot.pause(0.5)

#stop when figure is closed 
closeRequested = False
def onFigureClose(event):
    global closeRequested
    print('Figure closed!')
    closeRequested = True
pyplot.gcf().canvas.mpl_connect('close_event', onFigureClose)

#create video
filename = os.path.splitext(file.name)[0] + ".mp4"
print("start video file", filename)
writer = FFMpegFileWriter(fps=25, codec='h264', bitrate=5000)
writer.setup(fig, filename)

print("start populating")
startTime = perf_counter_ns()
lastPrintTime = perf_counter_ns()
frame = 0
lastFrameCount = 0
logDataIndex = 1
while not closeRequested and logDataIndex < len(logData):
    frame += 1
    videoTimeSecs = frame / TARGET_FPS

    while logDataIndex < len(logData):
        logDataRow = logData[logDataIndex]
        logTimeSecs = logDataRow[0]
        if X_MAX == 0:
            if logTimeSecs > videoTimeSecs:
                break
        else:
            if logTimeSecs - X_MIN > videoTimeSecs:
                break

        #add point to graph
        xValues.append(logTimeSecs)
        for i in range(1,len(logDataRow)):
            yValuesArray[i-1].append(logDataRow[i])

        logDataIndex += 1
    
    #update graph
    for i in range(len(plots)):
        plot = plots[i]
        plot.set_xdata(xValues)
        plot.set_ydata(yValuesArray[i])
    if X_MAX == 0:
        pyplot.axis([X_MIN + videoTimeSecs, X_MAX + videoTimeSecs, Y_MIN, Y_MAX])

    #refresh
    #pyplot.pause(0.001)
    pyplot.draw()
    fig.canvas.flush_events()
    writer.grab_frame()

    if (perf_counter_ns() - lastPrintTime) / 1e9 >= 2.0:
        secondsSinceLastPrint = (perf_counter_ns() - lastPrintTime) / 1e9
        lastPrintTime = perf_counter_ns()

        frameSaveSpeed = (frame - lastFrameCount) / secondsSinceLastPrint
        lastFrameCount = frame
        logInterval = (xValues[len(xValues) - 1] - xValues[len(xValues) - 2]) * 1000
        progress = len(xValues) / len(logData) * 100

        print("frameSaveSpeed: %.2f, logInterval: %.2f ms, progress: %.2f %%"
                % (frameSaveSpeed, logInterval, progress))

    sleep(0.001)

#write video file
print("finish video file", filename)
saveStartTime = perf_counter_ns()
writer.finish()
print("save lasted: %i ms" % ((perf_counter_ns() - saveStartTime) / 1e6))
print("finished")
