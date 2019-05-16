import time

import pigpio


#https://cdn.sparkfun.com/assets/b/3/0/b/a/DGCH-RED_datasheet.pdf
class hcsr04:

    def __init__(self, pi, trigger, echo, pulse_len = 15):

        self.pi = pi
        self.trigger = trigger
        self.echo = echo
        self.pulse_len = pulse_len
        self.tick_high = None
        self.tick_high_old = None
        self.tick_low = None
        self.tick_low_old = None
        self.pulse_width = None

        #http://abyz.me.uk/rpi/pigpio/python.html#set_mode
        self.pi.set_mode(gpio = self.trigger, mode = pigpio.OUTPUT)
        self.pi.set_mode(gpio = self.echo, mode = pigpio.INPUT)

        #http://abyz.me.uk/rpi/pigpio/python.html#callback
        self.cb = self.pi.callback(user_gpio=self.echo, edge=pigpio.EITHER_EDGE, func=self.cbf)

    #calculate the duty cycle
    def cbf(self, gpio, level, tick):

        #change to low (falling edge)
        if level == 0:
            
            self.tick_low = tick
            #try is needed because one of t1 or t2 might not be an int and then tickDiff would fail
            try:
                #http://abyz.me.uk/rpi/pigpio/python.html#callback
                # tick        32 bit    The number of microseconds since boot
                #                       WARNING: this wraps around from
                #                       4294967295 to 0 roughly every 72 minutes
                #Tested: This is handled by the tickDiff function internally, if t1 (earlier tick)
                #is smaller than t2 (later tick), which could happen every 72 min. The result will
                #not be a negative value, the real difference will be properly calculated.
                self.pulse_width = pigpio.tickDiff(t1=self.tick_high, t2=self.tick_low)
            except Exception:
                pass
            
        #change to high (rising edge)
        elif level == 1:

            self.tick_high = tick

    def trig(self):

        #default pulse_len is 50% more than needed (15 microseconds instead of 10)
        #to have a buffer to surely trigger the measurement
        self.pi.gpio_trigger(user_gpio = self.trigger, pulse_len = self.pulse_len, level = 1)

    def read(self, temp_air = 20, upper_limit = 4, number_of_sonic_bursts = 8, added_buffer = 2, debug = False):
        
        #speed of sound at 20 degree celsius -> c_air = 343.42 m/s
        c_air = 331.3 + (0.606 * temp_air)

        #max distance is upper_limit (in m):
        #let the sensor make the measurements, it will do a 8 cycle sonic burst:
        #
        #so 8 cycle sonic burst should take in worst 8 * 0,023 s = 0,184 s
        #so adding 100% buffer would lead to ~0,368 s
        #but that also depends on the used sonar sensor, see datasheet.
        #in this time frame the callback function should be called two times for each sonic burst.
        wait_for_measurement = upper_limit * 2 / c_air * number_of_sonic_bursts * added_buffer
        
        #counter while loop
        a = 0

        #check if recognized rising and falling edges are valid, so correctly catched by the callback function
        while self.tick_high is None or self.tick_high is self.tick_high_old or self.tick_low is None or self.tick_low is self.tick_low_old or self.pulse_width is None:
            
            self.trig()
            time.sleep(wait_for_measurement)
            #debugging information
            if a >= 1 and debug:
                print('{} {} {} {}'.format('number of extra measurements:', a, 'at this time:', time.time()))
            a=+1

        #calculated distance in m
        distance = self.pulse_width / 1000000 * c_air / 2

        #and store the the last values of self.tick_high and self.tick_low
        self.tick_high_old = self.tick_high
        self.tick_low_old = self.tick_low
        #set self.tick_high and self.tick_low back to initial values
        self.tick_high = None
        self.tick_low = None
        #set self.pulse width back to initial value
        self.pulse_width = None

        #check if measured/calculated distance is out of measurement range of the sensor,
        #see datasheet
        if distance >= upper_limit:

            if debug:

                print('{} {}'.format('out of upper limit, calculated distance:', distance))

            distance = upper_limit

        return distance
    
    def cancel(self):

        self.cb.cancel()

#https://www.parallax.com/sites/default/files/downloads/900-00005-Standard-Servo-Product-Documentation-v2.2.pdf

class para_standard_servo:

    def __init__(self, pi, gpio, min_pw = 1000, max_pw = 2000, min_degree = -90, max_degree = 90):

        self.pi = pi
        self.gpio = gpio
        #min_pw and max_pw might needed to be interchanged, depending on
        #if min_pw is moving servo to max_right or max_left,
        #see test functions below
        self.min_pw = min_pw
        self.max_pw = max_pw
        #allowed range of servo movement, see max_left(), max_right() functions
        self.min_degree = min_degree
        self.max_degree = max_degree
        #calculate slope for calculating the pulse width
        self.slope = (self.min_pw - ((self.min_pw + self.max_pw)/2)) / self.max_degree
        #calculate y-offset for calculating the pulse width
        self.offset = (self.min_pw + self.max_pw)/2

    def set_pw(self, pulse_width):
        
        #be carefully with setting the pulsewidth!!!
        #test carefully min_pw and max_pw value before settting them!!!
        #this can DAMAGE the servo!!!
        #http://abyz.me.uk/rpi/pigpio/python.html#set_servo_pulsewidth
        pulse_width = max(min(self.max_pw, pulse_width), self.min_degree)

        self.pi.set_servo_pulsewidth(user_gpio = self.gpio, pulsewidth = pulse_width)
        
    def calc_pw(self, degree):

        pulse_width = self.slope * degree + self.offset
        
        return pulse_width

    def set_position(self, degree):
        
        degree = max(min(self.max_degree, degree), self.min_degree)

        calculated_pw = self.calc_pw(degree = degree)
        self.set_pw(pulse_width = calculated_pw)

    def middle_position(self):

        #defines 0 degree
        pulse_width = (self.min_pw+self.max_pw)/2
        self.set_pw(pulse_width = pulse_width)

    def max_left(self):
        
        #defines -90 degree (min_degree)
        self.set_pw(self.max_pw)

    def max_right(self):

        #defines +90 degree (max_degree)
        self.set_pw(self.min_pw)

class scanner:

    #default values are for the CS installation used
    def __init__(self, pi, trigger = 6, echo = 5, gpio = 22, min_pw = 600, max_pw = 2350, angles = [-90, -45, 0, 45, 90]):

        #create one pigpio.pi() instance for the sensor and servo in parallel     
        self.pi = pi
        self.trigger = trigger
        self.echo = echo
        self.gpio = gpio
        self.min_pw = min_pw
        self.max_pw = max_pw
        self.angles = angles

        #initialize sonar and servo instance
        #https://gpiozero.readthedocs.io/en/stable/recipes.html#pin-numbering
        self.sonar = hcsr04(pi = self.pi, trigger = self.trigger, echo = self.echo)
        self.servo = para_standard_servo(pi = self.pi, gpio = self.gpio, min_pw = self.min_pw, max_pw = self.max_pw)

        #buffer time for initializing everything
        time.sleep(1)

    def read_all_angles(self, time_servo_reach_position = 0.35, debug = False):

        #create an empty dict
        measurement_dict = dict()
        if debug:
            start_time = time.time()
        #return servo to middle position, to not have to move e.g. from
        #last 90 degree to new -90 degree in following for loop
        self.servo.middle_position()
        #wait for servo reaching the position
        time.sleep(time_servo_reach_position)

        for ang in self.angles:

            self.servo.set_position(degree = ang)
            #wait for servo reaching the position
            time.sleep(time_servo_reach_position)
            measurement_dict[ang] = self.sonar.read(debug = debug)
        
        if debug:
            stop_time = time.time() - start_time
            print('{} {}'.format('time needed for one measurement round:', stop_time))
        return measurement_dict

    def cancel(self):

        #http://abyz.me.uk/rpi/pigpio/python.html#callback
        self.sonar.cancel()

if __name__ == '__main__':

    #### Example 1
    # pi = pigpio.pi()
    # ranger = scanner(pi = pi)
    # distances = ranger.read_all_angles()
    # print(distances)
    # #invoking this in the end is important
    # ranger.cancel()
    # pi.stop()

    #### Burn in test
    # while True:
    #     pi = pigpio.pi()
    #     ranger = scanner(pi = pi)
    #     distances = ranger.read_all_angles(debug = True)
    #     print(distances)
    #     #invoking this in the end is important
    #     ranger.cancel()
    #     pi.stop()

    #### Test constant measuring tow times
    pi = pigpio.pi()
    ranger = scanner(pi = pi)
    a = 0
    while a < 2:
        distances = ranger.read_all_angles(debug = True)
        print(distances)
        #invoking this in the end is important
        a += 1
    ranger.cancel()
    pi.stop()

    #### Example 2
    # pi = pigpio.pi()
    # servo = para_standard_servo(gpio = 22, pi = pi, min_pw = 600, max_pw = 2350)
    # servo.middle_position()
    # time.sleep(1)
    # servo.max_right()
    # time.sleep(1)
    # servo.max_left()
    # time.sleep(1)
    # servo.set_position(degree = 45)
    # pi.stop()
