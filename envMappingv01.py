import lib_scanner
import lib_motion
import pigpio
import time
import math

import numpy as np
import matplotlib.pyplot as plt

#Number of Particles
nPart = 50
#Robot position
rPos = [25.0, 25.0, 90.0]
rPosOld = [-10.0, 10.0, 180.0]

#Mean of error in x,y,theta movement
mean = [0.0 ,0.0, 0.0]

#Covariance of x, y, theta
covXY = [[0.13, 0.0,    0.0],
         [0,    0.37,   0],
         [0,    0,      2.74]]

def updateParticles(matPart, u):

    for item in range(nPart):
        eX, eY, eT = np.random.multivariate_normal(mean, covXY)

        part[item][0] = part[item][0] + u[0] + eX
        part[item][1] = part[item][1] + u[1] + eY
        part[item][2] = part[item][2] + u[2] + eY

        plt.plot([part[item][0]], [part[item][1]], 'r.')

def plotLines(xo, yo, x1, y1):
    #Lower from (-100,-10) to (10, -10)
    plt.plot([xo, x1], [yo, y1], 'k-', lw=1)
    
    
#Goal coordinates for navigation
wpNav = [[-30,30],
         [-50,10],
         [-50,41],
         [-100,30],
         [-90,0]]


#Plot target points
#plt.plot([-30, -50, -50, -100, -90], [30, 10, 41, 30, 0], 'ro')

#Axis Theoretical exterior of maze in cm
plt.axis([-50, 650, -50, 450])


#Plot contour of map
#Lower from (0,0) to (600, 0)
plt.plot([0, 600], [0, 0], 'k-', lw=2)
#Upper from (0,400) to (600, 400)
plt.plot([0, 600], [400, 400], 'k-', lw=2)
#Left from (0,0) to (0, 400)
plt.plot([0, 0], [0, 400], 'k-', lw=2)
#Right from (600,0) to (600, 400)
plt.plot([600, 600], [0, 400], 'k-', lw=2)


#Particles initialization
part = []
for item in range(nPart):
    new = [rPos[0],rPos[1], rPos[2]]
    part.append(new)

#Movement vectors u in cm and degrees
u =[    [6*25.0,90.0],
        [4*25.0,180.0],
        [10.0,90.0],
        [10.0,90.0],
        [10.0,90.0],
        [10.0,180.0],
        [10.0,270.0],
        [10.0,270.0],
        [10.0,270.0]
        ]

#Build uN array
uN = []
for j in range(len(u)):
    new = [ u[j][0]*math.cos(math.radians(u[j][1])), u[j][0]*math.sin(math.radians(u[j][1])), rPos[2]]
    uN.append(new)

#Plot initial position
plt.plot([rPos[0]], [rPos[1]], 'rx')

#Move a lot
for i in range(len(uN)):
    #Command Movement

    #Update particle position
    updateParticles(part, uN[i])

    #Theorical new position
    plt.plot([ rPos[0] + uN[i][0] ], [ rPos[1] + uN[i][1] ], 'co')
    #Update robot position
    rPosOld[0] = rPos[0]
    rPosOld[1] = rPos[1]
    rPosOld[2] = rPos[2]
    #Robot position
    rPos = [rPos[0] + uN[i][0], rPos[1] + uN[i][1], 180.0]

    plotLines(rPosOld[0], rPosOld[1], rPos[0], rPos[1])

#Show all
plt.show()
