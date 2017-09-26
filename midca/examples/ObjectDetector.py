'''
Created on Jun 16, 2015

@author: baxter
'''
#!/usr/bin/env python

import roslib; 

import rospy

import time
import  cv2
import numpy as np
from sys import argv
from homography import *
from baxter import *
from geometry_msgs.msg import Point, PointStamped
# Global variables:

H = []    # The current homography matrix.
Z = 0    # The Z coordinate of the table.
baxter = Baxter() # The object that controls Baxter. Defined in baxter.py.
floor_reference_points = [] # The floor reference points.
floor_reference_orientations = [] # The floor reference orientations.
n_clicks = 0
tot_clicks = 4
points = []

original_position = None
current_position = None
filename = "~/ros_ws/src/baxter_srv/scripts/calibration.txt"

def initial_setup_baxter():
    """
    Enable and set up baxter.

    """
    
    print 'Initializing node...'
    rospy.init_node('baxter_or')
    baxter.enable()
    baxter.calibrateLeftGripper()
    


def get_floor_reference_points():
    """
    This function get 4 points of reference from the real world, asking the
    user to move the baxter arm to the position of each corresponding point
    in the image, and then getting the X,Y and Z coordinates of baxter's hand.
    Returns an array of size 4 containing 4 coordinates:
    [[x1,y1], [x2,y2], [x3,y3], [x4,y4]].
    All the coordinates Z should be approximatelly the same. We assume the table
    is niveled. Save the Z coordinate in the global variable.

    TODO: Implement this. Figure out a way to get the end position of baxter
    hand. I know that in baxter_msgs

    """
    global Z # This declaration is needed to modify the global variable Z
    global floor_reference_points # Maybe erase.
    global floor_reference_orientations # Maybe erase.
    
    raw_input('Move the LEFT arm to point 1 and press enter.')
    p1 = baxter.getLeftArmPosition()
    o1 = baxter.getLeftArmOrientation()
    print o1
    #[-0.04504873726153096, 0.9983659324956183, 0.020392051958720396, -0.028639837992011533]
    print 'Point 1 =', p1
    raw_input('Move the LEFT arm to point 2 and press enter.')
    p2 = baxter.getLeftArmPosition()
    o2 = baxter.getLeftArmOrientation()
    print 'Point 2 =', p2
    raw_input('Move the LEFT arm to point 3 and press enter.')
    p3 = baxter.getLeftArmPosition()
    o3 = baxter.getLeftArmOrientation()
    print 'Point 3 =', p3
    raw_input('Move the LEFT arm to point 4 and press enter.')
    p4 = baxter.getLeftArmPosition()
    o4 = baxter.getLeftArmOrientation()
    print 'Point 4 =', p4
      

    Z = (p1[2] + p2[2] + p3[2] + p4[2]) / 4

    print Z

    
    filename = "/home/baxter/git/midca/examples/_baxter/calibration.txt"
    
    target = open(filename, 'w')

   
    target.truncate()
    
    target.write(repr(p1[0])+ " " + repr(p1[1]))
    target.write("\n")
    target.write(repr(p2[0])+ " " + repr(p2[1]))
    target.write("\n")
    target.write(repr(p3[0])+ " " + repr(p3[1]))
    target.write("\n")
    target.write(repr(p4[0])+ " " + repr(p4[1]))
    target.write("\n")
    
    
    target.close()
    
    
    return [[0.45835412247904794,   0.4167330917312844],
     [0.7046556740624649,   0.45390428836232344],
     [0.7778487250094798, 0.07406413897305184],
     [0.5418466718761972, 0.034360381218309734]]
    


def calibrate_homography():
    global H, Hinv
    floor_points = get_floor_reference_points()

        
def main():
    initial_setup_baxter()
    calibrate_homography()

    



if __name__ == '__main__':
    main()