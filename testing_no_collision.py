import lib_scanner
import lib_motion
import pigpio
import time

#initialize a pigpio.pi() instance to be used by all lib_*
pi = pigpio.pi()

robot = lib_motion.motion(pi = pi)
ranger =lib_scanner.scanner(pi = pi)

while True:

    distances = ranger.read_all_angles()
    print(distances)
    list_dist = list(distances.values())
    if any(t<0.4 for t in list_dist):
        robot.turn(45)

    elif any(t>=0.4 for t in list_dist):
        robot.straight(200)

robot.cancel()
ranger.cancel()
pi.stop()


### FIND OUT WHY IT ROTATES AND DRIVES TWO TIMES MORE THAN WANTED IN ONE TEST RUN
### And write down that edges are problems and are not detected because of physics
#die reflektieren dann weg

#document how much jitter we have and how many microseconds this is realted to