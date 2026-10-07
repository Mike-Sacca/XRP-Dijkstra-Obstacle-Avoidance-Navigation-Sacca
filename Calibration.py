from XRPLib.defaults import *

import time

from XRPLib . defaults import left_motor , right_motor , reflectance

# Description : This is a simple calibrator program to test sensors. Useful for torubleshooting and best practices. 
# Using this calibrator other systems can easily be recalibrated to work on diferently colored floor patterns.

# Inputs : N/A

# Outputs : N/A

while 1 == 1:
    print("RightValue")
    print(reflectance.get_right())
    print("LeftValue")
    print(reflectance.get_left())
    print("Rangefinder value")
    print(rangefinder.distance())
    time.sleep(0.5)
    
