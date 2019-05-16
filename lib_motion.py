import math
import statistics
import time

import pigpio

import lib_para_360_servo

class motion:

    #robot local coordinate system: x-positive straight forward, y-positive 
    #perpendicular to x to the direction to left wheel, center (0/0) where 
    #middle of robot chassis cutting across a imaginary line through both wheel 
    #axis. angle phi is the displacement of the local coordinate system to 
    #the real world coordinate system.

    #width robot: 102 mm (middle right wheel to middle left wheel, measured)
    #diameter wheels: 66mm (measured + data sheet)

    #default values are measured with the calibrate_pwm function from lib_servo.py
    def __init__(
        self, pi, width_robot = 102, diameter_wheels = 66, unitsFC = 360,
        dcMin_l = 27.3, dcMax_l = 969.15,
        dcMin_r = 27.3, dcMax_r = 978.25,
        l_wheel_gpio = 16, r_wheel_gpio = 20,
        servo_l_gpio = 17, min_pw_l = 1280, max_pw_l = 1720,
        servo_r_gpio = 27, min_pw_r = 1280, max_pw_r = 1720):
        
        self.pi = pi
        self.unitsFC = unitsFC
        self.dcMin_l = dcMin_l
        self.dcMax_l = dcMax_l
        self.dcMin_r = dcMin_r
        self.dcMax_r = dcMax_r
        self.width_robot = width_robot
        self.diameter_wheels = diameter_wheels

        self.l_wheel = lib_para_360_servo.read_pwm(pi = self.pi, gpio = l_wheel_gpio)
        self.r_wheel = lib_para_360_servo.read_pwm(pi = self.pi, gpio = r_wheel_gpio)
        self.servo_l = lib_para_360_servo.write_pwm(pi = self.pi, gpio = servo_l_gpio, min_pw = min_pw_l, max_pw = max_pw_l)
        self.servo_r = lib_para_360_servo.write_pwm(pi = self.pi, gpio = servo_r_gpio, min_pw = min_pw_r, max_pw = max_pw_r)

        #needed time for initializing the four instances
        time.sleep(1)

    #Angular position in units full circle
    def get_angle_l(self):

        #driving forward will increase the angle
        angle_l = (self.unitsFC - 1) - ((self.l_wheel.read() - self.dcMin_l) * self.unitsFC) / (self.dcMax_l - self.dcMin_l + 1)

        angle_l = max(min((self.unitsFC - 1), angle_l), 0)

        return angle_l

    #Angular position in units full circle
    def get_angle_r(self):

        #driving forward will increase the angle
        angle_r = (self.r_wheel.read() - self.dcMin_r) * self.unitsFC / (self.dcMax_r - self.dcMin_r + 1)

        angle_r = max(min((self.unitsFC - 1), angle_r), 0)

        return angle_r

    def set_speed_l(self, speed):

        self.servo_l.set_speed(-speed)

        return None

    def set_speed_r(self, speed):

        self.servo_r.set_speed(speed)

        return None

    def get_total_angle(self, angle, unitsFC, prev_angle, turns):
       
        #for counting number of rotations
        #If 4th to 1st quadrant
        if((angle < (0.25*unitsFC)) and (prev_angle > (0.75*unitsFC))):
            turns += 1
        #If in 1st to 4th quadrant
        elif((prev_angle < (0.25*unitsFC)) and (angle > (0.75*unitsFC))):
            turns -= 1

        #total angle measurement from zero
        if(turns >= 0):
            total_angle = (turns*unitsFC) + angle
        elif(turns < 0):
            total_angle = ((turns + 1)*unitsFC) - (unitsFC - angle)

        return turns, total_angle

    def get_target_angle(self, number_ticks, angle):
        
        #positiv number_ticks will be added, negativ number_ticks substracted
        target_angle = angle + number_ticks

        return target_angle

    def tick_length(self):

        tick_length_mm = math.pi * self.diameter_wheels / self.unitsFC

        return tick_length_mm

    def arc_circle(self, degree):

        arc_circle_mm = degree * math.pi * self.width_robot / 360.0

        return arc_circle_mm

    def turn(self, degree):

        number_ticks = self.arc_circle(degree)/self.tick_length()

        self.move(number_ticks = number_ticks, turn = True)

        return None

    def straight(self, distance_in_mm):

        number_ticks = distance_in_mm/self.tick_length()

        self.move(number_ticks = number_ticks, straight = True)

        return None

    def move(
        self,
        number_ticks = 0,
        #0.010 seconds:
        #1. PWM of motor feedback is 910Hz (0,001098901 s), so position changes cannot 
        #be recognized faster than 1.1 ms, therefore, it is not needed to run the outer control 
        #loop more often and update the speed values which have a 50 Hz (20ms) PWM.
        #2. Tests of the runtime of the code including the controller part showed, that
        #writing the pulse_width (pi.set_servo_pulsewidth()) in the lib_para_360_servo.py is 
        #the bottleneck which drastically slows down the code by the factor ~400 
        #(0,002 seconds ÷ 0,000005 seconds; runtime with / without writing pulsewidth).
        #3. For recognizing the RPMs of the wheel 10ms is needed to have enough changes in the
        #position, was found out by testing.
        sampling_time = 0.01,
        #PID-parameter
        Kp_p = 0.1, #not too fast, output of position control would slow down too abrupt otherwise
        Ki_p = 0,
        Kd_p = 0,
        Kp_s = 0.5,
        Ki_s = 0.1,
        Kd_s = 0,
        straight = False, turn = False):

        turns_l = 0
        turns_r = 0

        angle_l = self.get_angle_l()
        angle_r = self.get_angle_r()

        target_angle_r = self.get_target_angle(number_ticks = number_ticks, angle = angle_r)

        if straight == True:

            #speed and number_ticks of servo_l must rotate in
            #SAME direction to servo_r while driving straight
            target_angle_l = self.get_target_angle(number_ticks = number_ticks, angle = angle_l)

        elif turn == True:
            
            #speed and number_ticks of servo_l must rotate in
            #OPPOSITE direction to servo_r while turning
            target_angle_l = self.get_target_angle(number_ticks = -number_ticks, angle = angle_l)

        #initial values sum_error_*
        sum_error_r_p = 0
        sum_error_r_s = 0
        sum_error_l_p = 0
        sum_error_l_s = 0
        
        #initial values error_*_old
        error_l_p_old = 0
        error_l_s_old = 0
        error_r_p_old = 0
        error_r_s_old = 0

        #empty list for ticks_*
        list_ticks_l = []
        list_ticks_r = []

        #start time of the outer control loop
        start_time = time.time()
        #outer control loop: total distance/right wheel
        position_reached = False

        #### start of outer control loop: position right wheel
        while not position_reached:
            #DEBUGGING OPTION: printing runtime of loop , see end of while true loop
            #start_time_each_loop = time.time()

            angle_l = self.get_angle_l()
            angle_r = self.get_angle_r()

            try:
                turns_l, total_angle_l = self.get_total_angle(angle_l, self.unitsFC, prev_angle_l, turns_l)
                turns_r, total_angle_r = self.get_total_angle(angle_r, self.unitsFC, prev_angle_r, turns_r)

                #### Controller right wheel

                ## Position Controll
                #Er = SP - PV
                error_r_p = target_angle_r - total_angle_r
                #print(error_r_p)
                #Deadband-Filter to remove stuttering forward and backward after reaching the position
                if error_r_p <= 1 and error_r_p >= -1:
                    error_r_p = 0
                #I-Part
                sum_error_r_p += error_r_p
                #limit I-Part
                sum_error_r_p = max(min(20, sum_error_r_p), -20)

                #PI-Controller
                output_r_p = Kp_p * error_r_p + Ki_p * sampling_time * sum_error_r_p + Kd_p / sampling_time * (error_r_p - error_r_p_old)
                #limit output of position control to speed range
                output_r_p = max(min(1, output_r_p), -1)
                #print(output_r_p)

                error_r_p_old = error_r_p

                ## Speed Controll
                #convert range output_r_p from -1 to 1 to ticks/s
                output_r_p_con = 650 * output_r_p
                #ticks per second (ticks/s)
                ticks_r = (total_angle_r - prev_total_angle_r) / sampling_time
                list_ticks_r.append(ticks_r)
                list_ticks_r = list_ticks_r[-5:]
                ticks_r = statistics.median(list_ticks_r)
                
                #Er = SP - PV
                error_r_s = output_r_p_con - ticks_r

                #I-Part
                sum_error_r_s += error_r_s
                #limit I-Part
                #sum_error_r_s = max(min(650/Ki_s, sum_error_r_s), -650/Ki_s)

                #PI-Controller
                output_r_s = Kp_s * error_r_s + Ki_s * sampling_time * sum_error_r_s + Kd_s / sampling_time * (error_r_s - error_r_s_old)

                error_r_s_old = error_r_s

                #convert range output_r_s fom ticks/s to -1 to 1
                output_r_s_con = output_r_s / 650

                self.set_speed_r(output_r_s_con)

                #### Controller left wheel
                
                ## Position Control
                #Er = SP - PV
                error_l_p = target_angle_l - total_angle_l
                #Deadband-Filter to remove stuttering forward and backward after reaching the position
                if error_l_p <= 1 and error_l_p >= -1:
                    error_l_p = 0
                #I-Part
                sum_error_l_p += error_l_p

                #limit I-Part
                sum_error_l_p = max(min(20, sum_error_l_p), -20)

                #PI-Controller
                output_l_p = Kp_p * error_l_p + Ki_p * sampling_time * sum_error_l_p + Kd_p / sampling_time * (error_l_p - error_l_p_old)
                #limit output of position control to speed range
                output_l_p = max(min(1, output_l_p), -1)

                error_l_p_old = error_l_p

                ## Speed Controll
                #convert range output_r_p from -1 to 1 to ticks/s
                output_l_p_con = 650 * output_l_p
                #ticks per second (ticks/s)
                ticks_l = (total_angle_l - prev_total_angle_l) / sampling_time
                list_ticks_l.append(ticks_l)
                list_ticks_l = list_ticks_l[-5:]
                ticks_l = statistics.median(list_ticks_l)
                #Er = SP - PV
                error_l_s = output_l_p_con - ticks_l

                #I-Part
                sum_error_l_s += error_l_s
                #limit I-Part
                #sum_error_r_s = max(min(650/Ki_s, sum_error_r_s), -650/Ki_s)

                #PI-Controller
                output_l_s = Kp_s * error_l_s + Ki_s * sampling_time * sum_error_l_s + Kd_s / sampling_time * (error_l_s - error_l_s_old)

                error_l_s_old = error_l_s

                #convert range output_r_s fom ticks/s to -1 to 1
                output_l_s_con = output_l_s / 650

                self.set_speed_l(output_l_s_con)
                #self.set_speed_l(0.5)

            except Exception:
                pass

            prev_angle_l = angle_l
            prev_angle_r = angle_r

            #first run of while true loop, no total_angle_ is calcualted, because prev_angle_ is not available
            try:
                prev_total_angle_l = total_angle_l
                prev_total_angle_r = total_angle_r
            except Exception:
                pass

            try:
                if total_angle_r >= target_angle_r and number_ticks >= 0:

                    self.set_speed_r(0.0)
                    self.set_speed_l(0.0)
                    position_reached = True

                elif total_angle_r <= target_angle_r and number_ticks <= 0:
                    
                    self.set_speed_r(0.0)
                    self.set_speed_l(0.0)
                    position_reached = True

            except Exception:
                pass

            #Pause outer control loop to wait for changes in the system
            #https://stackoverflow.com/questions/474528/what-is-the-best-way-to-repeatedly-execute-a-function-every-x-seconds-in-python/25251804#25251804
            time.sleep(sampling_time - ((time.time() - start_time) % sampling_time))

            #DEBUGGING OPTION: printing runtime of loop, see beginning of while true loop
            #print('{:.20f}'.format((time.time() - start_time_each_loop)))
        
        return None

    def cancel(self):

        self.l_wheel.cancel()
        self.r_wheel.cancel()

if __name__ == '__main__':

    pi = pigpio.pi()

    robot = motion(pi = pi)

    a = 0
    while a < 4:
        robot.turn(45)
        time.sleep(1)
        a+=1

    robot.straight(500)
    time.sleep(1)
    robot.straight(-500)
    time.sleep(1)
    
    a = 0
    while a < 2:
        robot.turn(90)
        time.sleep(1)
        a+=1

    # while True:
    #     robot.straight(207.24)
    #     time.sleep(1)
    #     robot.straight(-207.24)
    #     time.sleep(1)
    
    #aaa = 0
    # for i in range(1):
    #     print('turn right')
    #     robot.turn(-180)
    #     time.sleep(1)
    #     print('turn left')
    #     robot.turn(180)
    #     time.sleep(1)
        #if aaa == 0:
        #    first = robot.l_wheel.duty_cycle
        #print('{} {} {}'.format('Measurement point', aaa, robot.l_wheel.duty_cycle))
        #aaa +=1
    #last = robot.l_wheel.duty_cycle
    #differences_both = first - last
    #print(differences_both)

    #http://abyz.me.uk/rpi/pigpio/python.html#callback
    robot.cancel()

    #http://abyz.me.uk/rpi/pigpio/python.html#stop
    pi.stop()
