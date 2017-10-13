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


def initial_setup_baxter():
    """
    Enable and set up baxter.

    """
    
    print 'Initializing node...'
    rospy.init_node('baxter_or')
    baxter.enable()
    baxter.calibrateLeftGripper()
    
   
def move_left_arm_to(point):
    origin = baxter.getLeftArmPosition()
    print 'origin =', origin
#     if origin[2] < Z + 0.2:
#         dest1 = origin[:]
#         dest1[2] += 0.2
#         print 'dest1 =', dest1
#         baxter.moveLeftArm(dest1, baxter.getLeftArmOrientation())
#         print 'done dest1'
#     if point[2] < Z + 0.1:
#         dest2 = point[:]
#         dest2[2] += 0.2
#         print 'dest2 =', dest2
#         baxter.moveLeftArm(dest2, baxter.getLeftArmOrientation())
#         print 'done dest2'
    print 'point =', point
    baxter.moveLeftArm(point, baxter.getLeftArmOrientation())
    print 'done point'

   
def test():
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
    #while n_clicks <= tot_clicks-1:
    cv2.imshow('image',cvimage)
    k = cv2.waitKey(0)
    if k == 27:         # wait for ESC key to exit
        cv2.destroyAllWindows()
        # displays the image
        #cv.ShowImage("Click", cvimage)
        #calls the callback function "on_mouse_click'when mouse is clicked inside window
        #cv.SetMouseCallback("Click", on_mouse_click, param=1)
        #cv.WaitKey(1000)
        
    
    #print points
    return points

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
    return points



def getObjectPosition():
    #raw_input('Enter to capture image.')
    image = baxter.getImageFromRightHandCamera()
    cvimage = baxter.getLastCvImage()
    
# Convert BGR to HSV
    hsv = cv2.cvtColor(cvimage, cv2.COLOR_BGR2HSV)
    
        # define range of red color in HSV
    lower_red = np.array([165,50,200])
    #0,60,60
    upper_red = np.array([175,200,255])
    #upper_red = np.array([175,200,255])
    #13,100,100
    
        # Threshold the HSV image to get only blue colors
    thresh = cv2.inRange(hsv, lower_red, upper_red)
    
        # Bitwise-AND mask and original image
    res = cv2.bitwise_and(cvimage,cvimage, mask= thresh)
    
#     cv2.imshow('frame',cvimage)
#     cv2.imshow('mask',thresh)
#     cv2.imshow('res',res)
#     k = cv2.waitKey(0) & 0xFF
#      
#     if k == 27:
#        cv2.destroyAllWindows()    
        
    image, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    bpoints1 = []
    bpoints2 = []
        
    if(len(contours) > 0):
        print len(contours) 
    else:
        print len(contours)
    
    if(len(contours) > 0):
        print len(contours)
#      
    
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
            #cv.Rectangle(cvimage,tuple(pt1),tuple(pt2), cv.CV_RGB(255,0,0), 1)
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
#             cv2.imshow('Click',cvimage)
#             cv2.waitKey(1000)
        #cv2.destroyAllWindows()
            return floor_point + [Z]
            
            
            #cv.Circle(cvimage,(centroidu,centroidv),5,0,-1)
            #floor_point = pixel_to_floor(H,[centroidu,centroidv])
            
            #cv2.destroyAllWindows()
            #return 0

    cv2.imshow('frame',frame)
    cv2.imshow('mask',thresh)
    #cv2.imshow('res',im)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        cv2.destroyAllWindows()

def getObjectPositionTest():
    raw_input('Enter to capture image.')
    image = baxter.getImageFromRightHandCamera()
    cvimage = baxter.getLastCvImage()
    
# Convert BGR to HSV
    hsv = cv2.cvtColor(cvimage, cv2.COLOR_BGR2HSV)
    
        # define range of red color in HSV
    lower_red = np.array([165,50,200])
    upper_red = np.array([175,200,255])
    
        # Threshold the HSV image to get only blue colors
    thresh = cv2.inRange(hsv, lower_red, upper_red)
    
        # Bitwise-AND mask and original image
    res = cv2.bitwise_and(cvimage,cvimage, mask= thresh)
    
#     cv2.imshow('frame',cvimage)
#     cv2.imshow('mask',thresh)
#     cv2.imshow('res',res)
#     k = cv2.waitKey(0) & 0xFF
     
    if k == 27:
       cv2.destroyAllWindows()    
        
    image, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
    if(len(contours) > 0):
        print len(contours) 
    else:
        print len(contours)
    
    


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
    
#     raw_input('Move the LEFT arm to point 1 and press enter.')
#     p1 = baxter.getLeftArmPosition()
#     o1 = baxter.getLeftArmOrientation()
#     print 'Point 1 =', p1
#     raw_input('Move the LEFT arm to point 2 and press enter.')
#     p2 = baxter.getLeftArmPosition()
#     o2 = baxter.getLeftArmOrientation()
#     print 'Point 2 =', p2
#     raw_input('Move the LEFT arm to point 3 and press enter.')
#     p3 = baxter.getLeftArmPosition()
#     o3 = baxter.getLeftArmOrientation()
#     print 'Point 3 =', p3
#     raw_input('Move the LEFT arm to point 4 and press enter.')
#     p4 = baxter.getLeftArmPosition()
#     o4 = baxter.getLeftArmOrientation()
#     print 'Point 4 =', p4
#      
#     floor_reference_points = [p1,p2,p3,p4]
#     floor_reference_orientations = [o1,o2,o3,o4]
    
 #    Calculate the z coordinate average:
    #Z = (p1[2] + p2[2] + p3[2] + p4[2]) / 4
    #Z = -0.19733055364191465
    Z = (-0.09213761966304572  -0.08460401925336455 -0.07989979719202103 -0.08507849515304883)/4
    print Z
# Point 1 = [0.5014114237373656, 0.2738107544597765, -0.09213761966304572]
# Move the LEFT arm to point 2 and press enter.
# Point 2 = [0.7535411194156703, 0.2897614820921219, -0.08460401925336455]
# Move the LEFT arm to point 3 and press enter.
# Point 3 = [0.7743015627913727, -0.12427782967004859, -0.07989979719202103]
# Move the LEFT arm to point 4 and press enter.
# Point 4 = [0.544968127426643, -0.15310413766573078, -0.08507849515304883]
# -0.0854299828154
    #return [[p1[0],p1[1]], [p2[0],p2[1]], [p3[0],p3[1]], [p4[0],p4[1]]]
    return [[0.5014114237373656, 0.2738107544597765],
     [0.7535411194156703, 0.2897614820921219],
      [0.7743015627913727, -0.124277829670048],
     [0.544968127426643, -0.15310413766573078]]


def calibrate_homography():
    global H, Hinv
    floor_points = get_floor_reference_points()
#     img_points = get_img_reference_points()
#     print img_points
#     print floor_points
    
    #H = homography_floor_to_img(img_points, floor_points)

def sendPoint(point):
    #rospy.init_node('baxter_grabbing')
    #x: 0.757512333588
#y: 0.0329546734934
#z: -0.078898709214)

    pub = rospy.Publisher('obj_pos', PointStamped, queue_size=10)
    points = [Point(x = point[0], y = point[1], z = point[2]),
              Point(x = 8.0, y = 7.0, z = 2.0),
              Point(x = 7.0, y = -4.0, z = -0.5), Point(x = -1, y = 1, z = 0)]
    if not rospy.is_shutdown():
        time.sleep(2)
        p = points[0]
        print("Sending point command:", p)
        pub.publish(PointStamped(point = p))
        
        
def main():
    initial_setup_baxter()
    calibrate_homography()
    position = getObjectPosition()
    print position
    #baxter.closeLeftGripper()
    #sendPoint(position)

if __name__ == '__main__':
    main()