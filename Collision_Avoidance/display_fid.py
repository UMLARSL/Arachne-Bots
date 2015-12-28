#!/usr/bin/python

# simple code to operate multiple spiders
import array
import time
import math
import sys
import os
import serial

# import Roborealm API Classes and fiducial functions
from RR_API import RR_API
import fiducials as fid

# instantiate RR_API class and connet to the API
# running on local computer
rr = RR_API()
rr.Connect("localhost")

# this needs to be changed to the correct COM port
# if it changes
#PORT = 'COM4'
#BAUD_RATE = 38400 # optimized for the panstamp

# create serial object to communicate with transmitter
#ser = serial.Serial(PORT, BAUD_RATE)

while True:
    try:

        #get camera frame rate
        fps = rr.GetVariable("frame_rate")

        # get FIDUCIALS and FIDUCIALS_PATH variables from RR_API
        fiducials = rr.GetFiducials()
        fiducialsPath = rr.GetFiducialsPath()

        # get orientation, position and name from every fiducial
        for key in fiducials.iterkeys():
            name = fid.get_fiducial_name(fiducials[key], fiducialsPath)
            
            if name == 'attic.gif':
                center_2 = fid.get_center(fiducials[key])
                spider = 0
                x_o = int(round(center_2[0]))
                y_o = int(round(center_2[1]))
                
            elif name == 'balcony.gif':
                spider = 8
                center = fid.get_center(fiducials[key])
                x = int(round(center[0]))
                y = int(round(center[1]))
                orientation = fid.get_orientation(fiducials[key])

        print "spider number: {0}".format(spider)
        print "x: {0}, y: {1}, angle: {2}".format(x, y, orientation)
        print "Time: {0}, FPS: {1}".format( time.time(), fps )

        time.sleep(.1)

    except ValueError:
        pass

    except KeyError:
        pass    

    except KeyboardInterrupt:
        print "all done"
        break

# close handle to transmitter object.
ser.close()
