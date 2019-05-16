import lib_scanner
import lib_motion
import pigpio
import time
import math
#Auxiliary functions

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


f = open("mapCoordinates.txt","r")

partMap = []
for x in f:
    #a , b = [x.strip() for item in x.split(',')]
    temp = x.split(',')
    newA = (float(temp[0]))
    newA = newA /10
    newAI = math.floor(newA)
    
    newB = (float(temp[1]))
    newB = newB /10
    newBI = math.floor(newB)
    
    new = [ newAI, newBI]
    partMap.append(new)
    print(new)


for item in range(len(partMap)):
    someX = partMap[item][0]
    someY = partMap[item][1]
    currentAxis = plt.gca()
    currentAxis.add_patch(Rectangle((someX- .1, someY-.1), .5, .5, angle=0.0))

    #rect = Rectangle((partMap[item][0],partMap[item][1]),5,5,fill=True)
#General map attributes
w, h = 40,30
mapMat = [[0 for x in range(w)] for y in range(h)]
#for item in range(len(partMap)):
#    print(item)
#    i = partMap[item][0]
#    print(i)
#    j = partMap[item][1]
#    print(j)
#    if i >=0  and j>0:
#        mapMat[i][j] = mapMat[i][j] + 1
    
print(mapMat[16][19])

#Axis Theoretical exterior of maze in cm
plt.axis([-5, 35, -5, 25])

#Plot contour of map
#Lower from (0,0) to (30, 0)
plt.plot([0, 30], [0, 0], 'k-', lw=2)

#Upper  from (0,20) to (30, 20)
plt.plot([0, 30], [20, 20], 'k-', lw=2)

#Left from (0,0) to (0, 20)
plt.plot([0, 0], [0, 20], 'k-', lw=2)

#Right from (30,0) to (30, 20)
plt.plot([30, 30], [0, 20], 'k-', lw=2)

plt.show()