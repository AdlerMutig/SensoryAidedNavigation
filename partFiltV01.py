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

def plotParticles(matPart):
    print(np.matrix(matPart))
    return 0


print('initializing ok')

if __name__ == '__main__':

    #Robot position
    rPos = [-10.0, 10.0, 180.0]


    #Goal coordinates for navigation
    wpNav = [[-30,30],
             [-50,10],
             [-50,41],
             [-100,30],
             [-90,0]]
    #Mean of error in x,y,theta movement
    mean = [0.0 ,0.0, 0.0]

    #Covariance of x, y, theta
    covXY = [[0.13, 0.0,    0.0],
             [0,    0.37,   0],
             [0,    0,      2.74]]

    x1, y1, z1 = np.random.multivariate_normal(mean, covXY)
    x2, y2, z2 = np.random.multivariate_normal(mean, covXY)

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

    #Movement vectors:
    uN = [  [-10.0, 0.0, 0.0],
            [-10.0, 0.0, 0.0],
            [0.0, 5.0, 0.0],
            [0.0, 5.0, 0.0]]
    u = [-10, 0 , 0]

    #Plot initial position
    plt.plot([rPos[0]], [rPos[1]], 'rx')

    #Command Movement

    #Particle position
    for item in range(nPart):
        eX, eY, eT = np.random.multivariate_normal(mean, covXY)
        print("error en x", eX)
        print("error en y", eY)
        print("error en t", eT)

        part[item][0] = part[item][0] + u[0] + eX
        part[item][1] = part[item][1] + u[1] + eY
        part[item][2] = part[item][2] + u[2] + eY

        plt.plot([part[item][0]], [part[item][1]], 'mo')
    #Theorical new position
    plt.plot([ rPos[0] + u[0] ], [ rPos[1] + u[1] ], 'cx')

    #Show all
    plt.show()
