import collections
import statistics
import time

import pigpio

#https://www.parallax.com/sites/default/files/downloads/900-00360-Feedback-360-HS-Servo-v1.1.pdf
#http://gpiozero.readthedocs.io/en/stable/remote_gpio.html
#https://gpiozero.readthedocs.io/en/stable/recipes.html#pin-numbering

class write_pwm:

    def __init__(self, pi, gpio, min_pw = 1280, max_pw = 1720, min_speed = -1, max_speed = 1):

        self.pi = pi
        self.gpio = gpio
        #min_pw and max_pw might needed to be interchanged, depending on
        #if min_pw is moving servo to max_forward or max_backward
        #see test functions below
        self.min_pw = min_pw
        self.max_pw = max_pw
        #define max and minimum speed
        self.min_speed = min_speed
        self.max_speed = max_speed
        #calculate slope for calculating the pulse width
        self.slope = (self.min_pw - ((self.min_pw + self.max_pw)/2)) / self.max_speed
        #calculate y-offset for calculating the pulse width
        self.offset = (self.min_pw + self.max_pw)/2

    def set_pw(self, pulse_width):
        
        #be carefully with setting the pulsewidth!!!
        #test carefully min_pw and max_pw value before settting them!!!
        #this can DAMAGE the servo!!!
        #http://abyz.me.uk/rpi/pigpio/python.html#set_servo_pulsewidth
        pulse_width = max(min(self.max_pw, pulse_width), self.min_pw)

        self.pi.set_servo_pulsewidth(user_gpio = self.gpio, pulsewidth = pulse_width)
        
    def calc_pw(self, speed):

        pulse_width = self.slope * speed + self.offset
        
        return pulse_width

    def set_speed(self, speed):

        speed = max(min(self.max_speed, speed), self.min_speed)

        calculated_pw = self.calc_pw(speed = speed)
        self.set_pw(pulse_width = calculated_pw)

    def stop(self):

        pulse_width = (self.min_pw+self.max_pw)/2
        self.set_pw(pulse_width = pulse_width)

    def max_forward(self):
        
        self.set_pw(self.min_pw)

    def max_backward(self):

        self.set_pw(self.max_pw)

#hardcoded period for 910 Hz Signal
class read_pwm:

    def __init__(self, pi, gpio):

        self.pi = pi
        self.gpio = gpio
        self.period = 1/910*1000000
        self.tick_high = None
        self.duty_cycle = None
        self.duty_scale = 1000

        #http://abyz.me.uk/rpi/pigpio/python.html#set_mode
        self.pi.set_mode(gpio=self.gpio, mode=pigpio.INPUT)
        #http://abyz.me.uk/rpi/pigpio/python.html#callback
        self.cb = self.pi.callback(user_gpio=self.gpio, edge=pigpio.EITHER_EDGE, func=self.cbf)

    #calculate the duty cycle
    def cbf(self, gpio, level, tick):
        #change to low (a falling edge)
        if level == 0:
            #if first edge is a falling one the following code will not work
            #a try first time is faster than an if-statement every time 
            try:
                self.duty_cycle = self.duty_scale*pigpio.tickDiff(t1=self.tick_high, t2=tick)/self.period

            except Exception:
                pass

        #change to high (a rising edge)
        elif level == 1:

            self.tick_high = tick

    def read(self):

        return self.duty_cycle

    #cancel the started callblack function http://abyz.me.uk/rpi/pigpio/python.html#callback
    def cancel(self):

        self.cb.cancel()

#servo speed 0.2 is needed for calibrating and is needed to be set "manually" 
#by setting the servo to this speed, see Example 1 below
#hardcoded period for 910 Hz Signal
class calibrate_pwm:

    def __init__(self, pi, gpio, measurement_time = 120):
         
        self.pi = pi
        self.gpio = gpio
        self.period = 1/910*1000000
        self.tick_high = None
        self.duty_cycle = None
        self.duty_scale = 1000
        self.list_duty_cycles = []
        self.duty_cycle_min = None
        self.duty_cycle_max = None

        #http://abyz.me.uk/rpi/pigpio/python.html#set_mode
        self.pi.set_mode(gpio=self.gpio, mode=pigpio.INPUT)

        #http://abyz.me.uk/rpi/pigpio/python.html#callback
        self.cb = self.pi.callback(user_gpio=self.gpio, edge=pigpio.EITHER_EDGE, func=self.cbf)
        
        #measurement time
        print('{}{}{}'.format('Starting measurements for: ', measurement_time, ' seconds.'))
        print('----------------------------------------------------------')
        time.sleep(measurement_time)

        #stop callback before sorting list to avoid getting added new elements unintended
        #http://abyz.me.uk/rpi/pigpio/python.html#callback
        self.cb.cancel()
        time.sleep(1)
        
        self.list_duty_cycles = sorted(self.list_duty_cycles)

        #some analyzis of the dc values
        sorted_set = list(sorted(set(self.list_duty_cycles)))
        print('{} {}'.format('Ascending sorted distinct duty cycle values:', sorted_set))
        print('----------------------------------------------------------')
        differences_list = [sorted_set[i+1]-sorted_set[i] for i in range(len(sorted_set)-1)]
        rounded_differences_list = [round(differences_list[i],2) for i in range(len(differences_list)-1)]
        counted_sorted_list = collections.Counter(rounded_differences_list)
        print('{} {}'.format('Ascending counted, sorted and rounded distinct differences between duty cycle values:',counted_sorted_list))
        print('----------------------------------------------------------')

        # Median is chosen, because the biggest and smallest values are needed, and not an
        # avarage of the smallest and biggest values of the selection. 
        # Values smaller and bigger than the printed out once as "duty_cycle_min/duty_cycle_max"
        # are outliers and should therefore not be considered. This can be seen in printout
        # of smallest/biggest 250 values. Compare the printouts to get a feeling for it and
        # also run it serveral times.
        #https://docs.python.org/3/library/statistics.html#statistics.median
        print('{} {}'.format('Smallest 250 values:', self.list_duty_cycles[:250]))
        self.duty_cycle_min = statistics.median_high(self.list_duty_cycles[:20])
        print('----------------------------------------------------------')
        print('{} {}'.format('Biggest 250 values:',self.list_duty_cycles[-250:]))
        self.duty_cycle_max = statistics.median_low(self.list_duty_cycles[-20:])
        print('----------------------------------------------------------')

        print('duty_cycle_min:', round(self.duty_cycle_min,2))

        print('duty_cycle_max:', round(self.duty_cycle_max,2))
        
    def cbf(self, gpio, level, tick):

        #change to low (a falling edge)
        if level == 0:
            #if first edge is a falling one the following code will not work
            #a try first time is faster than an if-statement every time 
            try:
                self.duty_cycle = self.duty_scale*pigpio.tickDiff(t1=self.tick_high, t2=tick)/self.period
                self.list_duty_cycles.append(self.duty_cycle)

            except Exception:
                pass

        #change to high (a rising edge)
        elif level == 1:

            self.tick_high = tick
            
    #cancel the started callblack function http://abyz.me.uk/rpi/pigpio/python.html#callback
    def cancel(self):

        self.cb.cancel()

if __name__ == "__main__":

    #define GPIO for each sensor to read from
    gpio_l_r = 16
    gpio_r_r = 20

    #define GPIO for each sensor to write to
    gpio_l_w = 17
    gpio_r_w = 27
    
    pi=pigpio.pi()

    #### Example 1 - Calibrate servos, speed  = 0.2 or -0.2
    # The measured values are needed in the lib_motion.py for each 360 degree servo.
    # chose gpio_l_w/gpio_l_r (left wheel) accordingly gpio_r_w/gpio_r_r (right wheel)
    # IMPORTANT, the robot wheels must be able to rotate free in the air for this.
    # ALSO IMPORTANT rotating forward or backward might sometimes give slightly 
    # different results! Chose the smalles values of forward and backward runs and the
    # biggest of forward and backward runs. Do both directions three times for each wheel.

    servo = write_pwm(pi = pi, gpio=gpio_r_w, min_pw = 1280, max_pw = 1720)
    #buffer time for initializing everything
    time.sleep(1)
    servo.set_speed(-0.2)
    wheel=calibrate_pwm(pi = pi, gpio = gpio_r_r)
    servo.set_speed(0)
    
    # #### Example 2 - How granular speed changes?
    #
    # servo = write_pwm(pi = pi, gpio=gpio_r_w, min_pw = 1280, max_pw = 1720)
    # #buffer time for initializing everything
    # time.sleep(1)
    # speed = 0
    # while speed < 1:
    #     servo.set_speed(speed)
    #     print('{} {}'.format('Set speed:', speed))
    #     time.sleep(1)
    #     speed += 0.025
    # servo.set_speed(0)

    #http://abyz.me.uk/rpi/pigpio/python.html#stop
    pi.stop()
