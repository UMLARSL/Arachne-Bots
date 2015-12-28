# some functions for dealing with fiducials

import math

# get_center grabs the four corners of the fiducial 
# and calculates a center point that is the average of those
def get_center(fiducial):
    xs = fiducial[1:8:2] # slice x coordinates
    ys = fiducial[2:9:2] # slice y coordinates
    center = [ (sum(xs) / len(xs)), (sum(ys) / len(ys)) ]
    return center

# this grabs the orientation from a fiducial array
# returned from roborealm.
def get_orientation(fiducial):
    orientation = fiducial[14] * (math.pi / 180)
    return orientation

# extract the fiducial's name given the index
# into roborealm's FIDUCIALS_PATH and the lengh of the name
def get_fiducial_name(fiducial, path):
    index = int(fiducial[15])
    length = int(fiducial[16])
    name = path[index:(length + index) - 1]
    return name

# extracts scale from fiducial array
def get_scale(fiducial):
    scale = float(fiducial[11])
    return scale
