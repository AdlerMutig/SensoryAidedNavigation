import lib_scanner
import lib_motion
import pigpio
import time

#initialize a pigpio.pi() instance to be used by all lib_*
pi = pigpio.pi()

robot = lib_motion.motion(pi = pi)
ranger =lib_scanner.scanner(pi = pi)
#Test move spin and meassure

distances = ranger.read_all_angles()
print(distances)
robot.turn(45)
robot.straight(20)#mm

robot.cancel()
ranger.cancel()
pi.stop()