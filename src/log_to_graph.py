import matplotlib.pyplot as pyplot

print("read from file")
file = open("datalog.dat", "r")
fileLines = file.readlines()
file.close()

print("parse data")
logData = []
for fileLine in fileLines:
    va = fileLine.strip().split(' ')
    if fileLine != fileLines[0]:
        va = [float(v) for v in va]
    logData.append(va)
xValues = []
yValuesArray = []
for i in range(1,len(logData[0])):
    yValuesArray.append([])
for logLine in logData[1:]:
    xValues.append(logLine[0]) #secondsSinceStart
    for i in range(1,len(logLine)):
        yValuesArray[i-1].append(logLine[i])

print("populate graph")
pyplot.xlabel('time [sec]')
for yValues in yValuesArray:
    pyplot.plot(xValues, yValues)
pyplot.legend(logData[0][1:])

print("show graph")
pyplot.show()
