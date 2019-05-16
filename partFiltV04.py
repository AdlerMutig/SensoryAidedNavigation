#import lib_servo
#import pigpio
import time
import math
#import statistics
#from gpiozero.pins.pigpio import PiGPIOFactory
#from gpiozero import DistanceSensor
##Particular implementation map
import matplotlib.pyplot as plt
import numpy as np
##End of particular implementation

#Number of Particles
nPart = 50
#Robot position
rPos = [-10.0, 10.0, 180.0]
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

print('initializing ok')

if __name__ == '__main__':




    #Goal coordinates for navigation
    wpNav = [[-30,30],
             [-50,10],
             [-50,41],
             [-100,30],
             [-90,0]]


    #Plot target points
    #plt.plot([-30, -50, -50, -100, -90], [30, 10, 41, 30, 0], 'ro')

    #Axis
    plt.axis([-130, 30, -20, 60])


    #Plot contour of map
    #Lower from (-100,-10) to (10, -10)
    plt.plot([-110, 10], [-10, -10], 'k-', lw=2)
    #Upper from (-100,50) to (10, 50)
    plt.plot([-110, 10], [50, 50], 'k-', lw=2)
    #Left from (-100,-100) to (50, -10)
    plt.plot([-110, -110], [-10, 50], 'k-', lw=2)
    #Right from (-100,-10) to (10, -10)
    plt.plot([10, 10], [50, -10], 'k-', lw=2)


    #Particles initialization
    part = []
    for item in range(nPart):
        new = [rPos[0],rPos[1], rPos[2]]
        part.append(new)

    #Movement vectors u in cm and degrees
    u =[    [20.0,180.0],
            [20.0,180.0],
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
