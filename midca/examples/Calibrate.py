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

from homography import *
from baxter import *
from geometry_msgs.msg import Point, PointStamped
from _dbus_bindings import String
from sys import argv
from decimal import *
# Global variables:

H = []    # The current homography matrix.
Z = 0    # The Z coordinate of the table.
baxter = Baxter() # The object that controls Baxter. Defined in baxter.py.
floor_reference_points = [] # The floor reference points.
floor_reference_orientations = [] # The floor reference orientations.
n_clicks = 0
tot_clicks = 4
points = []
filename = argv
original_position = None
current_position = None


def initial_setup_baxter():
    """
    Enable and set up baxter.

    """
    
    #print 'Initializing node...'
    #rospy.init_node('baxter_or')
    baxter.enable()
    baxter.calibrateLeftGripper()
    
   

   

def on_mouse_click(event, x, y, flag, param):
    global n_clicks, points
    if event == cv2.EVENT_LBUTTONDOWN:
        print 'Point %s captured: (%s,%s)' % (n_clicks+1,x,y)
        points.append([x, y])
        n_clicks += 1


def get_img_reference_points():
    """
    This function get 4 points of reference from the image from the right hand
    of baxter. Returns an array of size 4, with 4 coordinates:
    [[x1,y1], [x2,y2], [x3,y3], [x4,y4]].

    TODO: implement this. We have to define a color we will mark the table
    and get 4 points of that color from the image.

    """
    # The following line is just for test.
    raw_input('Enter to capture image.')
    image = baxter.getImageFromRightHandCamera()
    cvimage = baxter.getLastCvImage()
    while n_clicks <= tot_clicks-1:
        # displays the image
        cv2.imshow("Click", cvimage)
        #cv.ShowImage("Click", cvimage)
        #calls the callback function "on_mouse_click'when mouse is clicked inside window
        cv2.setMouseCallback("Click", on_mouse_click, param=1)
        #cv.SetMouseCallback("Click", on_mouse_click, param=1)
        #cv.WaitKey(1000)
        cv2.waitKey(1000)
    
    #print points
    cv2.destroyAllWindows()    
    return points


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
    
    #Z = (-0.04311285564353425  -0.04512672573083166 -0.04080078888404003 -0.046071914959185875)/4
    #Z= -0.04721129960500225
    Z = -0.15113003072395247
    print Z
# [0.5264201148167275, 0.40034933311487086, -0.027560670871152958]
# Point 1 = [0.5264201148167275, 0.40034933311487086, -0.027560670871152958]
# Move the LEFT arm to point 2 and press enter.
# Move the LEFT arm to point 3 and press enter.
# Point 3 = [0.8164126163781988, 0.00011724257622775782, -0.006060458646583389]
# Move the LEFT arm to point 4 and press enter.
# Point 4 = [0.5774338486223564, -0.02912627450728407, -0.02923769860966796]
# Point 1 = [0.45835412247904794, 0.4167330917312844, -0.11362745036843477]
# Move the LEFT arm to point 2 and press enter.
# Point 2 = [0.7046556740624649, 0.45390428836232344, -0.11322759071560898]
# Move the LEFT arm to point 3 and press enter.
# Point 3 = [0.7778487250094798, 0.07406413897305184, -0.11181591166991744]
# Move the LEFT arm to point 4 and press enter.
# Point 4 = [0.5418466718761972, 0.034360381218309734, -0.11464607923115094]

    #return [[p1[0],p1[1]], [p2[0],p2[1]], [p3[0],p3[1]], [p4[0],p4[1]]]
   
    #print p4
    filename = "/home/baxter/git/midca/examples/_baxter/calibration.txt"
    f = open(filename, 'r')
    p1 = f.readline().split(' ')
    p2 = f.readline().split(' ')
    p3 = f.readline().split(' ')
    p4 = f.readline().split(' ')
    
    p1[0] = float(p1[0])
    p1[1] = float(p1[1])
    p2[0] = float(p2[0])
    p2[1] = float(p2[1])
    p3[0] = float(p3[0])
    p3[1] = float(p3[1])
    p4[0] = float(p4[0])
    p4[1] = float(p4[1])
    
    return [[p1[0], p1[1]],
     [p2[0], p2[1]],
     [p3[0], p3[1]],
     [p4[0], p4[1]]]
    
#    return [[0.5773763528146585, 0.3842165517841408],
#     [0.7855928713464901, 0.37834930053240295],
#     [0.76618765321789, -0.02885636412309065],
#     [0.5568000497983868, -0.01377416902917198]]
    
def calibrate_homography():
    global H, Hinv
    floor_points = get_floor_reference_points()
    img_points = get_img_reference_points()
    print img_points
    print floor_points
    H = homography_floor_to_img(img_points, floor_points)
    return H
    #I need to send H in string format
    
def sendPoint(msg, topic):
    pub = rospy.Publisher(topic, String, queue_size=10)
    
    if not rospy.is_shutdown():
        time.sleep(2)
        pub.publish(msg)
        

def msg_as_string(H):
    HtoString=""
    Q = np.linalg.inv(H)
    HtoString = Q[0,0]
    HtoString = HtoString+","+Q[0,1]
    HtoString = HtoString+","+Q[0,2]
    HtoString = HtoString+","+Q[1,0]
    HtoString = HtoString+","+Q[1,1]
    HtoString = HtoString+","+Q[1,2]
    HtoString = HtoString+","+Q[2,0]
    HtoString = HtoString+","+Q[2,1]
    HtoString = HtoString+","+Q[2,2]        
    return HtoString
    
def calibrate():
    initial_setup_baxter()
    H = calibrate_homography()
    return H
    #sendPoint(msg_as_string(H), "calibrate_done")
#     position = getObjectPosition()
#     #baxter.closeLeftGripper()
#     sendPoint(position)

def getZ():
    global Z
    return Z
    
    
if __name__ == '__main__':
    calibrate()
