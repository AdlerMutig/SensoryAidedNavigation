import lib_scanner
import lib_motion
import pigpio
import time
import math
#Auxiliary functions

import numpy as np
import matplotlib.pyplot as plt

def angleCorrection(dx, dy):
    angRad = math.atan2(dY,dX)
    goal = math.degrees(angRad)
    if goal <0:
        goal = 360 + goal
    print("Goal angle", goal)
    return math.degrees(angRad)

def shortestSpin(pos,goal):
    changeAngle =  -pos + goal
    
    #Calculate Shortest spin
    if abs(changeAngle)>180:
        if changeAngle >0:
            changeAngle = changeAngle - 360.0 
        else:
            changeAngle =  360.0 + changeAngle
    return changeAngle

def plotPoints(distances):
    
    p0   = [rPos[0] + math.cos(math.radians(rPos[2]))*distances[0]*100,     rPos[1]+ math.sin(math.radians(rPos[2]))*distances[0]*100]
    p90  = [rPos[0] + math.cos(math.radians(rPos[2]-90))*distances[90]*100,   rPos[1]+ math.sin(math.radians(rPos[2]-90))*distances[90]*100]
    pm45 = [rPos[0] + math.cos(math.radians(rPos[2]+45))*distances[-45]*100, rPos[1]+ math.sin(math.radians(rPos[2]+45))*distances[-45]*100]
    p45  = [rPos[0] + math.cos(math.radians(rPos[2]-45))*distances[45]*100,   rPos[1]+ math.sin(math.radians(rPos[2]-45))*distances[45]*100]
    pm90 = [rPos[0] + math.cos(math.radians(rPos[2]+90))*distances[-90]*100, rPos[1]+ math.sin(math.radians(rPos[2]+90))*distances[-90]*100]
    
    addParticles(p0)
    
    addParticles(p90)
    
    addParticles(pm90)
    
    addParticles(p45)
    
    addParticles(pm45)
    
    
    #plt.plot(p0[0],p0[1],color='green', marker='o')
    
    #plt.plot(p90[0],p90[1],color='red', marker='o')
    
    #plt.plot(pm45[0],pm45[1],color='blue', marker='o')
    
    #plt.plot(p45[ƒRPOSR],p45[1],color='yellow', marker='o')
    
    #plt.plot(pm90[0],pm90[1],color='gray', marker='o')
    

def addParticles(u):
    
    for item in range(nPartUS):
        eX, eY = np.random.multivariate_normal(meanUS, covUS)

        partUS[item][0] = u[0] + eX
        partUS[item][1] = u[1] + eY
        
        new = [partUS[item][0], partUS[item][1] ]
        mapPart.append(new)
        
        plt.plot([partUS[item][0]], [partUS[item][1]], 'r.')
    
#initialize a pigpio.pi() instance to be used by all lib_*
pi = pigpio.pi()

robot = lib_motion.motion(pi = pi)
ranger =lib_scanner.scanner(pi = pi)

#Mean of error in x,y of ultrasound meassurement mean
meanUS = [0.0 ,0.0]

#Number of Particles for sensor meassure
nPartUS = 5

#Covariance of x, y position of sensor measure
covUS = [[0.23, 0.0], #Error measured 
          [0,    0.06]] #Calculated from spin error variation

#Map particles

mapPart = []
#Sensor Particles initialization
partUS = []
for item in range(nPartUS):
    new = [0,0]
    partUS.append(new)


#Robot global position
rPos = [25.0, 75.0, 90.0]
rPosOld = [25.0, 25.0, 90.0]

#Mean of error in x,y,theta movement
mean = [0.0 ,0.0, 0.0]

#Covariance of x, y, theta
covXY = [[0.13, 0.0,    0.0],
         [0,    0.37,   0],
         [0,    0,      2.74]]

#CorrectionAngle
corrAng = -2

#Goal coordinates for navigation
wpNav = [[25,175]]


#wpNav = [[25,175],
#         [125,175],
#         [125,75],
#         [225,75]]

#wpNav = [[25,100],
#         [75,100],
#         [75,125]]

#General map attributes

#Axis Theoretical exterior of maze in cm
plt.axis([-50, 350, -50, 250])


#Plot contour of map
#Lower from (0,0) to (300, 0)
plt.plot([0, 300], [0, 0], 'k-', lw=2)

#Upper 1 from (0,200) to (150, 200)
plt.plot([0, 150], [200, 200], 'k-', lw=2)

#Upper 2 from (150,100) to (200, 100)
plt.plot([150, 200], [100, 100], 'k-', lw=2)

#Upper 3 from (200,150) to (300, 150)
plt.plot([200, 300], [150, 150], 'k-', lw=2)

#Left from (0,0) to (0, 200)
plt.plot([0, 0], [0, 200], 'k-', lw=2)

#Right from (300,0) to (300, 150)
plt.plot([300, 300], [0, 150], 'k-', lw=2)

#Mid1  from (150,100) to (150, 200)
plt.plot([150, 150], [100, 200], 'k-', lw=2)

#Mid 2 from (200,100) to (200, 150)
plt.plot([200, 200], [100, 150], 'k-', lw=2)

#Initial read
distances = ranger.read_all_angles()
print(distances)
plotPoints(distances)

#iterate over points
i=0
while True:
    #Plot robot trayectory position
    plt.plot([rPos[0]], [rPos[1]], 'bx')
    
    if distances[0]> .30 and distances[-45]> .3 and distances[45]>.3:
        if distances[90] < .15:
            wpNav[i] = [rPos[0]+10*math.cos(math.radians(135)),rPos[1]+ 10*math.cos(math.radians(135))]
            print("back in track to the left")
            
        else:
            if distances[-90] < .15:
                wpNav[i] = [rPos[0]+10*math.cos(math.radians(45)),rPos[1]+ 10*math.cos(math.radians(45))]
                print("back in track to the right")
                
            else:
                wpNav[i] = [rPos[0],rPos[1]+ 15]
                print("forward")
    else:
        if distances[90]> .3 and distances[45] >.3:
            wpNav[i]= [rPos[0]+15,rPos[1]]
            print("move right")
        else:
            if distances[-90]> .3 and distances[-45]>.3:
                wpNav[i] = [rPos[0]-15,rPos[1]]
                print("move left")
        
    
    print("target coordinate", wpNav[i])
    print("current angle ",  rPos[2])
    #Get movement deltas
    dX = wpNav[i][0] - rPos[0]
    dY = wpNav[i][1] - rPos[1]
    
    #Calculate angle between points and convert to deg
    angDegC = angleCorrection(dY,dX)

    
    #Calculate Spin
    changeAngle = shortestSpin(rPos[2], angDegC)
    print("should spin ",  changeAngle)
    
    #Calculate Distance
    dist = 10*math.sqrt(dX**2 + dY**2)
    print("should drive ",  dist)
    
    #Execute movement
    robot.turn(changeAngle)
    #robot.turn(corrAng)
    time.sleep(1)
    i  =i +1
    #while dist>80:
    #    robot.straight(80)
    #    dist = dist-80
    #    robot.turn(corrAng)
    #    time.sleep
    robot.straight(dist)
    time.sleep(1)
    
    #Update predicted position
    rPos[0] = wpNav[i][0]
    rPos[1] = wpNav[i][1]
    #Display only positive angles
    if angDegC <0:
        angDegC = 360 + angDegC
    rPos[2] = angDegC
    
    #Measure distances in new position
    distances = ranger.read_all_angles()
    print(distances)
    plotPoints(distances)
    
        

#Show all
f = open("mapCoordinates.txt","a")
for item in range(len(mapPart)):
    temp = ''
    temp= temp + str(mapPart[item][0])+","+ str(mapPart[item][1])+"\n"
    f.write(temp)
f.close()
plt.show()
robot.cancel()
ranger.cancel()
pi.stop()