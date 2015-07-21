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
# Global variables:


#H = []    # The current homography matrix.
Z = 0    # The Z coordinate of the table.
baxter = Baxter() # The object that controls Baxter. Defined in baxter.py.
floor_reference_points = [] # The floor reference points.
floor_reference_orientations = [] # The floor reference orientations.
n_clicks = 0
tot_clicks = 4
points = []

original_position = None
current_position = None

def initial_setup_baxter():
    """
    Enable and set up baxter.

    """
    baxter.enable()
    

def getObjectPosition(H, Z):
   
    image = baxter.getImageFromRightHandCamera()
    cvimage = baxter.getLastCvImage()
    
    # Convert BGR to HSV
    hsv = cv2.cvtColor(cvimage, cv2.COLOR_BGR2HSV)
    
    lower_red = np.array([0,60,60])
    #0,60,60
    upper_red = np.array([10,100,100])
   
        # Threshold the HSV image to get only blue colors
    thresh = cv2.inRange(hsv, lower_red, upper_red)
    
        # Bitwise-AND mask and original image
    res = cv2.bitwise_and(cvimage,cvimage, mask= thresh)
        
    image, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    bpoints1 = []
    bpoints2 = []
        
    if(len(contours) > 0):
        print len(contours) 
    else:
        return None
    
    cnt = contours[0]
    
    if(len(cnt) > 0):
        #print len(contours)
        cnt = contours[0]
        M = cv2.moments(cnt)
        print M
        
        x,y,w,h = cv2.boundingRect(cnt)
        pt1 = [x, y]
        pt2 = [x + w, y + h]
        
        bpoints1.append(pt1)
        bpoints2.append(pt2)
        
        z = np.array([])        
    
        if np.size(bpoints1) > 0:
            #bpoints1 is top left, bpoints 2 is bottom right
            pt1max = np.array([0,0])
            pt2max = np.array([0,0])
    
            for i in range(len(bpoints1)):
                maxdiag = np.linalg.norm(pt2max - pt1max)
        
                p1 = np.array([bpoints1[i][0],bpoints1[i][1]])
                p2 = np.array([bpoints2[i][0],bpoints2[i][1]])
                diag = np.linalg.norm(p2 - p1)
                if diag > maxdiag:
                    pt1max = p1
                    pt2max = p2
            
            # UPPER LEFT CORNER OF THE LARGEST RECTANGLE
            pt1 = pt1max
            # LOWER RIGHT CORNER OF THE LARGEST RECTANGLE
            pt2 = pt2max
            cvimage = cv2.rectangle(cvimage,tuple(pt1),tuple(pt2),(0,255,0),2)
            # HORIZON COORDINATE OF THE LARGEST RECTANGLE
            centroidu = (pt1[0]+pt2[0])/2
            # VERTICAL COORDINATE OF THE LARGEST RECTANGLE
            centroidv = (pt1[1]+pt2[1])/2
            
            (x,y),radius = cv2.minEnclosingCircle(cnt)
            center = (int(x),int(y))
            radius = int(radius)
            #frame = cv2.circle(frame,(centroidu,centroidv),(0,255,0),2)
            cvimage = cv2.circle(cvimage,center,radius,(0,255,0),2)
            
            floor_point = pixel_to_floor(H,[centroidu,centroidv])
            return floor_point + [Z]
            

def sendPoint(point):

    pub = rospy.Publisher('obj_pos', PointStamped, queue_size=10)
    points = [Point(x = point[0], y = point[1], z = point[2]),
              Point(x = 8.0, y = 7.0, z = 2.0),
              Point(x = 7.0, y = -4.0, z = -0.5), Point(x = -1, y = 1, z = 0)]
    if not rospy.is_shutdown():
        time.sleep(2)
        p = points[0]
        print("Sending point command:", p)
        pub.publish(PointStamped(point = p))
        
        
def main(H, Z):
    #print(H)
    initial_setup_baxter()
    #calibrate_homography()
    position = getObjectPosition(H, Z)
    #baxter.closeLeftGripper()
    #sendPoint(position)
    if(position != None):
        p = Point(x = position[0], y = position[1], z = position[2])
    
        return PointStamped(point = p)
    
    return None

if __name__ == '__main__':
    main()