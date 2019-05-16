import lib_scanner
import lib_motion
import pigpio
import time
import math

import numpy as np
import matplotlib.pyplot as plt

y = [i for i in range(20,100,3)]
x = [i for i in range(len(y))]

plt.scatter(x,y)
plt.show()