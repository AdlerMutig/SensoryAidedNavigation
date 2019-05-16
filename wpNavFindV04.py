#Own robot libraries
import lib_scanner
import lib_motion
import pigpio

#General Libraries
import time
import math
import json

#Threading libraries
from threading import Thread
from time import sleep

#Gyro
from mpu6050 import mpu6050

#Plotting librarie
import numpy as np
import matplotlib.pyplot as plt
##########
#Constants
##########

#Drift offset
drift = -1.29811

#Stored distances to known walls
storedP = [ [1.73684665,	2.740189,	  0.24640385],# 0, 7, 12
            [1.7394223,	 1.69391915,	1.24575605],# 1, 11
            [1.2225752,	 1.1933845,	 0.25498935],# 2, 10
            [0.74264575,	0.20193096,	1.2500488], # 3
            [0.21206185,	0.70315245,	0.25670645],# 4
            [0.1991836,	 0.20175925,	0.75809965],# 5
            [1.24592776,	0.1940323,	 2.75440011],# 6
            [100,	       100,	       100],# 7 holder
            [0.7332017,	 0.71826293,	0.23764664],# 8
            [0.25670645,	0.25670645, 	1.25670645]] # 9
 
pointL = [[25.0,  25.0],  #0
          [125.0, 25.0],  #1
          [125.0, 75.0],  #2
          [225.0, 75.0],  #3
          [225.0, 125.0], #4
          [275.0, 125.0], #5
          [275.0, 25.0],  #6
          [25.0,  25.0],  #7
          [25.0,  125.0], #8
          [125.0, 175.0], #9
          [125.0, 75.0],  #10
          [125.0, 25.0],  #11
          [25.0,  25.0]]  #12

wpNav = [ [25,25],                  #0
        [125, 25,  175, 175, 25,  0],   #1
        [125, 75,  25,  125, 125, 90],  #2
        [225, 75,  75,  25,  25,  0],   #3
        [225, 125, 25,  25,  75,  90],   #4
        [275, 125, 25,  25, 125,  0],   #5
        [275, 25,  25,  25, 275,  270],   #6
        [25,  25,  25,  25, 175,  180],   #7
        [25,  125, 25,  75, 75,   90],    #8
        [125, 175, 25,  25, 175,  0],   #9
        [125, 75,  125, 75, 25,   270],    #10
        [125, 25, 175,25, 125,    270],     #11
        [25,  25, 25,25, 175,     180]]      #12

def substractLists(a, b):
    diff = 0
    for item in range(len(a)):
        diff = diff + abs(a[item]-b[item])
    return diff

def substractListsB(a, b):
    diff = 0
    diff = [a[0]-b[0], a[1]-b[1], a[2]-b[2]]
    return diff

def determinePosition():
    min = 99999
    index = -1
    #median list
    medL0   = []
    medL90  = []   
    medmL90 = []
    #Measure distances 5 times
    for item in range(1):
        current = ranger.read_all_angles()
        print("distancias medidas")
        print(current[0],current[90],current[-90],)
        medL0.append(current[0])
        medL90.append(current[90])
        medmL90.append(current[-90])

    #Calculate mean
    med0 =   np.median(np.array(medL0))
    med90 =  np.median(np.array(medL90))
    medm90 = np.median(np.array(medmL90))
    
    #median list
    medL = [med0,med90,medm90]
    print("medianas callculadas con esas medidas", medL)
    #compare with all points
    
    for i in range(9):
        diff = substractLists(medL, storedP[i])
        if diff < min:
            min = diff
            index = i
            
    
    diffL = substractListsB(medL, storedP[index])
    print("diferencia entre lo teorico y lo medido, error")
    print(diffL)
    if diffL[0] > 0.4 or diffL[1] > .4 or diffL[2] >.4:
        input("Ultrasound sensor error detected")
    
    return index
     

#Trigonometric and Navigation Functions
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

def addParticles(u):
    
    for item in range(nPartUS):
        eX, eY = np.random.multivariate_normal(meanUS, covUS)

        partUS[item][0] = u[0] + eX
        partUS[item][1] = u[1] + eY
        
        new = [partUS[item][0], partUS[item][1] ]
        mapPart.append(new)
        
        plt.plot([partUS[item][0]], [partUS[item][1]], 'r.')

#Orientation calculation thread function
def readGyro(arg,movTime):
    
    listGyro = []
    gyroMed = 0
    deg = 0
    for i in range(movTime+2):
        gyro_data = sensor.get_gyro_data()

        #print("Gyroscope data")
        sleep(.1)
        listGyro.append((gyro_data['z']-drift)*.1)
        listGyro = listGyro[-5:]
        gyroMed = gyroMed + np.median(listGyro)
               
    f = open("rotation.txt", "w")
    f.write(str(gyroMed))
    f.close
    return deg

#initialize a pigpio.pi() instance to be used by all lib_*
pi = pigpio.pi()

robot = lib_motion.motion(pi = pi)
ranger =lib_scanner.scanner(pi = pi)

#Initialize Gyroscope
sensor = mpu6050(0x68)
print("waiting for sensor to calibrate")
sleep(2)

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



#Read current position
pos = determinePosition()
origen = pos

#Position was determined
print("Origin position: ", origen)

#Robot global position
rPos = [pointL[origen][0], pointL[origen][1], 90.0]
rPosOld = [pointL[origen][0], pointL[origen][1], 90.0]

first = True


#iterate over points
for i in range((pos+1),len(wpNav)):
  
    #Plot robot trayectory position
    #plt.plot([rPos[0]], [rPos[1]], 'rx')

    print("Target coordinate", wpNav[i])
    print("Current angle ",  rPos[2])
    
    #Get movement deltas
    dX = wpNav[i][0] - rPos[0]
    dY = wpNav[i][1] - rPos[1]
    
    #Calculate angle between points and convert to deg
    angDegC = angleCorrection(dY,dX)

    #Calculate Spin
    changeAngle = shortestSpin(rPos[2], angDegC)
    print("Should spin ",  changeAngle)
    
    #Calculate Distance
    dist = 10*math.sqrt(dX**2 + dY**2)
    print("Should drive ",  dist)
    
    #Start thread for rotation calculation
    
    thread1 = Thread(target = readGyro, args = (10,20, ))
    thread1.start()
    
    #Execute movement
    robot.turn(changeAngle)
    thread1.join()
    time.sleep(1)
    
    
    with open("rotation.txt", "r") as ins:
        for line in ins:
            angleCorr= float(line)
    print("Angle correction")
    robot.turn(changeAngle -( -angleCorr))
    sleep(1)
    #while dist>80:
    #    robot.straight(80)
    #    dist = dist-80
    #    robot.turn(corrAng)
    #    time.sleep
    
    thread1 = Thread(target = readGyro, args = (10,int(dist/10), ))
    thread1.start()
    
    robot.straight(dist)
    time.sleep(1)
    thread1.join()
    print("exit thread")
    
    with open("rotation.txt", "r") as ins:
        for line in ins:
            angleCorr= float(line)
    print("va a girar para corregir", -angleCorr)
    
    robot.turn(0-(-angleCorr))
    time.sleep(1)
    
    #Update predicted position
    rPos[0] = wpNav[i][0]
    rPos[1] = wpNav[i][1]

    #Store only positive angles
    if angDegC <0:
        angDegC = 360 + angDegC
    rPos[2] = angDegC

    #Measure distances in new position
    distances = ranger.read_all_angles()
 
    p0   = distances[0]*100
    p90  = distances[90]*100  
    
    if wpNav[i][5]==0:
        rPos[0] = rPos[0] - (wpNav[i][4]-p0)  
        rPos[1] = rPos[1] - (wpNav[i][5]-p90)
    
    if wpNav[i][5]==90:
        rPos[0] = rPos[0] - (wpNav[i][5]-p90)
        rPos[1] = rPos[1] - (wpNav[i][4]-p0)
    
    if wpNav[i][5]==180:
        rPos[0] = rPos[0] + (wpNav[i][4]-p0)
        rPos[1] = rPos[1] - (wpNav[i][5]-p90)
    
    if wpNav[i][5]==270:
        rPos[0] = rPos[0] + (wpNav[i][5]-p90)
        rPos[1] = rPos[1] - (wpNav[i][4]-p0)  
    


    print("fin del ciclo")
    #Measure distances in new position
    #distances = ranger.read_all_angles()
    #print(distances)
    #plotPoints(distances)
    
        

#Show all
f = open("mapCoordinates.txt","a")
for item in range(len(mapPart)):
    temp = ''
    temp= temp + str(mapPart[item][0])+","+ str(mapPart[item][1])+"\n"
    f.write(temp)

f.close()
#pplt.show()
robot.cancel()
ranger.cancel()
pi.stop()