#!/usr/bin/python

"""
    The program to connect roborealm and the spiders.
    The script feeds the modem panstamp the coordinates and
    destinations of each spider individually.
    This version corrects errors that cause the fiducial array
    to send coordinates based on fiducial confidence
    rather than a particular fiducial.
    
    Destinations are calculated using the collision avoidance algorithm.
    The spiders should be implementing just the simple movement algorithm.
    Changes the way that addresses are assigned so that it is more dynamic.
    The script also creates a log file for each spider in the program.
    make sure to denote the number of spiders being used as N.

    ***IMPORTANT***
    Make sure all coordinates transmitted over serial are formatted as ints
    
"""
    

import array
import time as t
import datetime
import math
import os
import serial
from collision_avoidance import *
from collections import defaultdict
from RR_API import RR_API
import fiducials as fid
from numpy import *
import numpy as np

rr = RR_API()
rr.Connect("localhost")

# this needs to be changed to the          x5                   6                   x7                   8                   x9                   10correct COM port
PORT = 'COM9'
BAUD_RATE = 38400
ser = serial.Serial(PORT, BAUD_RATE)

"""print "Place all spiders being used into the camera's view"
for i in range(5):
    t.sleep(1)
    print str( 5 - i )

#get fiducials from roborealm
data = rr.GetFiducialsPath()
fiducials = rr.GetFiducials()
"""
#dict for determining default array index of fiducials in vision
#to add more fiducial, check their filename using the roborealm 'display fiducial name' option
index = { 'arrow.gif' : 0, 'bathroom.gif' : 1, 'balcony.gif' : 2, 'garden.gif' : 3, 'sdfsdf.gif' : 8, 'asdfasdf.gif' : 10}
#           x5                   6                   x7                   8                   x9                   10

#count the spiders
N = 4
#for key in sorted( fiducials.iterkeys() ):
 #   N += 14

#create list to convert index to address
options = [ x*2 + 7 for x in range( N ) ]

print str(N) + " Spiders detected."

#get time for timestamp
time = datetime.datetime.now()
stamp = str(time.hour) + "-" + str(time.minute)

log = range( N )#create a list of logs of data for each spider
#open each log and label the variables
for x in log:
    log[x] = open( "spider_" + str(x) + "_data_" + stamp + ".csv", "w" )
    log[x].write( "Spider, Index, X_d, Y_d, X, Y, Theta, Elapsed Time(s)\n" )

poses = np.zeros( ( N, 3 ) ) #create array of spider positions
targets = np.array( [[156, 277], [470, 273], [253,338], [200,200]] ) #array of targets for all spiders

print "Spider Targets Are: "
print targets

print"Beginning in:"
for i in range(3):
    t.sleep(1)
    print str( 3 - i )
print "\n"

#main operations loop
while True:
    try:
        # get all variables from RR_API
        data = rr.GetFiducialsPath()
        fiducials = rr.GetFiducials()

        #get poses and assign them to each spider
        for key in sorted( fiducials.iterkeys() ):    
            orientation = fid.get_orientation( fiducials[key] )
            center = fid.get_center( fiducials[key] )
                
            
            #decode fiducials by name into index number
            name = fid.get_fiducial_name( fiducials[key], data )
            i = index[ name ]

            #take out
          #  if i%2 == 0:
           #     i = i - 1
            
            #populate array with current positions of spiders iteratively
            poses[i, 0] = int( round( center[0] ) ) #spider x
            poses[i, 1] = int( round( center[1] ) ) #spider y
            poses[i, 2] = orientation

            
        print "Poses:"
        print poses
        print "Setpoints:"   
        setpoints = getSetpoints( N, poses, targets )#get array of adjusted destinations for all spiders
        print setpoints
        
        #determine spider to send data to.  includes assignment of radio address by array index
        for spiderIndex in range( N ):
            #assign adjusted destination coordinates to spiders

            
            #packet defined as: start sequence, address, x_d, y_d, x, y, theta
            #write packet to modem on serial
            ser.write( "-./{0},{1},{2},{3},{4},{5}+++".format( options[spiderIndex],
                                                            int( setpoints[ spiderIndex][0] ),
                                                            int( setpoints[ spiderIndex][1] ),
                                                            int( poses[spiderIndex][0] ),
                                                            int( poses[spiderIndex][1] ),
                                                            int( poses[spiderIndex][2] * 1000 ) ))

            
            #write data to log files
##            log[spiderIndex].write( "{0},{1},{2},{3},{4},{5},{6},{7}\n".format( options[spiderIndex],
##                                                            spiderIndex,
##                                                            setpoints[ spiderIndex, 0],
##                                                            setpoints[ spiderIndex, 1],
##                                                            poses[spiderIndex, 0],
##                                                            poses[spiderIndex, 1],
##                                                            poses[spiderIndex, 2],
##                                                            t.clock() ) )
            
            ser.write( "-./{0},{1},{2},{3},{4},{5}+++".format( options[spiderIndex],
                                                            int( setpoints[ spiderIndex][0] ),
                                                            int( setpoints[ spiderIndex][1] ),
                                                            int( poses[spiderIndex][0] ),
                                                            int( poses[spiderIndex][1] ),
                                                            int( poses[spiderIndex][2] * 1000 ) ))

                
           
            

    except KeyError:
        pass

    except KeyboardInterrupt:
        print "\n\nAll Done.\n\n"
        break



#close serial and log files
for x in range( N ):
    log[x].close()
    
ser.close()
