import lib_servo
import pigpio
import time
import math
#import statistics
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import DistanceSensor

#http://gpiozero.readthedocs.io/en/stable/_modules/gpiozero/pins/pigpio.html?highlight=PiGPIOFactory
#http://gpiozero.readthedocs.io/en/stable/remote_gpio.html
#http://gpiozero.readthedocs.io/en/stable/api_pins.html
#select pigpio as driver for the GPIO pins
factory = PiGPIOFactory()

servo_r = lib_servo.write_pwm(gpio=14, min_pulse_width=1.380/1000, max_pulse_width=1.620/1000, frame_width=20/1000, factory = factory)
servo_l = lib_servo.write_pwm(gpio=15, min_pulse_width=1.380/1000, max_pulse_width=1.620/1000, frame_width=20/1000, factory = factory)

r_wheel=lib_servo.read_pwm(pi=pigpio.pi(), gpio=23)
l_wheel=lib_servo.read_pwm(pi=pigpio.pi(), gpio=24)

#define as GPIO number (echo and trigger)
sensor_left = DistanceSensor(echo=8, trigger=16, max_distance=1, pin_factory=factory)
sensor_front = DistanceSensor(echo=7, trigger=20, max_distance=1, pin_factory=factory)
sensor_right = DistanceSensor(echo=5, trigger=21, max_distance=1, pin_factory=factory)

class motion:

    #robot local coordinate system: x-positive straight forward, y-positive perpendicular to x to the direction to left wheel, center (0/0) where middle of robot chassis cutting across a imaginary line through both wheel axis. angle phi is the displacement of the local coordinate system to the real world coordinate system.

    #width robot: 103 mm (middle right wheel to middle left wheel)
    #diameter wheels: 66mm (measured + data sheet)

    #default values are measured with the calibrate_pwm function from lib_servo.py or with a folding rule for this specific robot
    def __init__(self, unitsFC_r = 64, dcMin_r = 3.15, dcMax_r = 97.29, unitsFC_l = 64, dcMin_l = 2.74, dcMax_l = 97.25, width_robot = 103, diameter_wheels = 66, r_wheel = r_wheel, l_wheel = l_wheel, servo_r = servo_r, servo_l = servo_l):
        
        self.unitsFC_r = unitsFC_r
        self.dcMin_r = dcMin_r
        self.dcMax_r = dcMax_r
        self.unitsFC_l = unitsFC_l
        self.dcMin_l = dcMin_l
        self.dcMax_l = dcMax_l
        self.width_robot = width_robot
        self.diameter_wheels = diameter_wheels

        self.r_wheel = r_wheel
        self.l_wheel = l_wheel
        self.servo_r = servo_r
        self.servo_l = servo_l

    def get_angle_r(self):

        angle_r = (self.r_wheel.duty_cycle - self.dcMin_r) * self.unitsFC_r / (self.dcMax_r - self.dcMin_r + 1)

        return angle_r

    def get_angle_l(self):

        angle_l = (self.unitsFC_l - 1) - ((self.l_wheel.duty_cycle - self.dcMin_l) * self.unitsFC_l) / (self.dcMax_l - self.dcMin_l + 1)

        return angle_l

    def set_speed_r(self, speed):

        self.servo_r.servo.value = -speed

        return None

    def set_speed_l(self, speed):

        self.servo_l.servo.value = speed

        return None

    def count_turns_and_total_angle(self, int_angle, unitsFC, prev_int_angle, turns):
       
        #for counting number of rotations
        #If 4th to 1st quadrant
        if((int_angle < (0.25*unitsFC)) and (prev_int_angle > (0.75*unitsFC))):
            turns += 1
        #If in 1st to 4th quadrant
        elif((prev_int_angle < (0.25*unitsFC)) and (int_angle > (0.75*unitsFC))):
            turns -= 1

        #total angle measurement from zero
        if(turns >= 0):
            total_angle = (turns*unitsFC) + int_angle
        elif(turns < 0):
            total_angle = ((turns + 1)*unitsFC) - (unitsFC - int_angle)

        return turns, total_angle

    def forward_or_backward(self, int_number_ticks, speed, int_angle):

        if int_number_ticks >= 0:
            target_angle = int_angle + int_number_ticks

            return target_angle, speed

        elif int_number_ticks < 0:
            target_angle = int_angle - abs(int_number_ticks)

            return target_angle, -speed

    def move(self, speed = 0.3, number_ticks = 12, position_reached = False, straight = False, turn = False):

        int_number_ticks = int(number_ticks)
        turns_r = 0
        turns_l = 0
        list_total_angles_r = []
        list_total_angles_l = []
        counter_change_speed = 0

        int_angle_r = int(self.get_angle_r())

        target_angle_r, speed = self.forward_or_backward(int_number_ticks, speed, int_angle_r)

        if straight == True:
            self.set_speed_r(speed)
            self.set_speed_l(speed)
        elif turn == True:
            self.set_speed_r(speed)
            self.set_speed_l(-speed)
        else:
            print('Please choose if to move straight or to turn')
            position_reached = True

        while not position_reached:

#IMPORTANT to get just integers AND to get for values slighlty below zero (e.g. -0.02) a zero and for values slightly above 63 (e.g. 63.23) a 63 to get 64 distinct values
            int_angle_r = int(self.get_angle_r())
            int_angle_l = int(self.get_angle_l())

            try:
                turns_r, total_angle_r = self.count_turns_and_total_angle(int_angle_r, self.unitsFC_r, prev_int_angle_r, turns_r)
                turns_l, total_angle_l = self.count_turns_and_total_angle(int_angle_l, self.unitsFC_l, prev_int_angle_l, turns_l)

            except:
                time.sleep(0)

############################## implementation of a speed control

            #speed of each wheel
            try:
                list_total_angles_r.append(total_angle_r)
                list_total_angles_l.append(total_angle_l)

            except:
                time.sleep(0)

            #try:
                #if list_total_angles_r[-500]>0.0 and list_total_angles_r[-1]>0.0 and list_total_angles_l[-500]>0.0 and list_total_angles_l[-1]>0.0:
                    #if (list_total_angles_r[-500] - list_total_angles_r[-1]) > (list_total_angles_l[-500] - list_total_angles_l[-1]):
                        # +- 0.01666666... seems to be the smallest amount the speed has to change, smaller values are not possible (regarding some tests, 0.4 base speed * 1.04 does not change the speed, base speed * 1.05 is the smallest value where changes happen)
                        #if not (counter_change_speed % 1000):
                            #self.set_speed_l(self.servo_l.servo.value*1.05)
                            #print(counter_change_speed)
                            #print('Speed left wheel: ' + str(self.servo_l.servo.value))

                        #print('Speed right wheel: ' + str(self.servo_r.servo.value))
                        #print('Speed left wheel: ' + str(self.servo_l.servo.value))
                        #counter_change_speed += 1
                    #if (list_total_angles_r[-50] - list_total_angles_r[-1]) < (list_total_angles_l[-50] - list_total_angles_l[-1]):
                        #self.set_speed_l(self.servo_l.servo.value*0.99)


            #except:
                #time.sleep(0)
            #print('Speed left wheel: ' + str(self.servo_l.servo.value))
################################

            prev_int_angle_r = int_angle_r
            prev_int_angle_l = int_angle_l

            try:
                if total_angle_r >= target_angle_r and speed >= 0:
                    self.set_speed_r(0.0)
                    self.set_speed_l(0.0)
                    #stop while loop
                    position_reached = True
                elif total_angle_r <= target_angle_r and speed <= 0:
                    self.set_speed_r(0.0)
                    self.set_speed_l(0.0)
                    #stop while loop
                    position_reached = True
            except:
                time.sleep(0)
        
        return None

    def tick_length(self):

        tick_length = math.pi * self.diameter_wheels / self.unitsFC_r

        return tick_length

    def arc_circle(self, degree):

        arc_circle = degree * math.pi * self.width_robot / 360.0

        return arc_circle

    def turn(self, speed, degree):

        number_ticks = int(self.arc_circle(degree)/self.tick_length())

        self.move(speed = speed, number_ticks = number_ticks, turn = True)

        return None

    def straight(self, speed, distance_in_mm):

        number_ticks = int(distance_in_mm/self.tick_length())

        self.move(speed = speed, number_ticks = number_ticks, straight = True)

        return None
    
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

#needed time for initializing everything
time.sleep(1)

print('initializing ok')

if __name__ == '__main__':

    #move 10 ticks straight (needed to be calculated to milimeter for movement and rotation to degree! MISSING!!!!!

    robot = motion()
    vel = 0.5
    #Robot global position
    rPos = [0.0, 0.0, 180.0]
    
    #Goal coordinates for navigation
    wpNav = [[-30,30],
             [-50,10],
             [-50,41],
             [-100,30],
             [-90,0],
             [0,0]]
    
    #iterate over points
    for i in range(len(wpNav)):
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
        robot.turn(vel, changeAngle)
        time.sleep(1)
        robot.straight(vel,dist)
        time.sleep(1)
        
        #Update predicted position
        rPos[0] = wpNav[i][0]
        rPos[1] = wpNav[i][1]
        #Display only positive angles
        if angDegC <0:
            angDegC = 360 + angDegC
        rPos[2] = angDegC
        
        

    #
    #
    #robot.turn(0.5, -45)



    #http://abyz.me.uk/rpi/pigpio/python.html#callback
    r_wheel.cancel()
    l_wheel.cancel()

    #http://abyz.me.uk/rpi/pigpio/python.html#stop
    r_wheel.pi.stop()
    l_wheel.pi.stop()


    #robot.turn(0.3,90)
    #robot.straight(0.3,50)
    #left lane
#    robot.straight(0.4,845)
#    robot.turn(0.4,-90)
#    robot.straight(0.4,375)
#    robot.turn(0.4,90)
#    robot.straight(0.4,250)
#    robot.turn(0.4,-90)
#    robot.straight(0.4,375)
#    robot.turn(0.4,-90)
#    robot.straight(0.4,1175)
#    robot.turn(0.4,-90)
#    robot.straight(0.4,470)

    #right lane
#    robot.straight(0.4,495)
#    time.sleep(1)
#    robot.turn(0.4,-90)
#    time.sleep(1)
#    robot.straight(0.4,300)
#    time.sleep(1)
#    robot.turn(0.4,90)
#    time.sleep(1)
#    robot.straight(0.4,345)
#    time.sleep(1)
#    robot.turn(0.4,-90)
#    time.sleep(1)
#    robot.straight(0.4,310)
#    time.sleep(1)
#    robot.turn(0.4,-90)
#    time.sleep(1)
#    robot.straight(0.4,840)
#    time.sleep(1)
#    robot.turn(0.4,-90)
#    time.sleep(1)
#    robot.straight(0.4,610)
