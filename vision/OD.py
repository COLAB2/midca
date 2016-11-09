import cv2
import numpy as np
from _dbus_bindings import String

cap = cv2.VideoCapture(0)
from baxter import *
import time
baxter = Baxter()

import os
import sys
import argparse
     
import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

from sensor_msgs.msg import (
    Image,
)


# green = np.uint8([[[0,255,0 ]]])
# hsv_green = cv2.cvtColor(green,cv2.COLOR_BGR2HSV)
# print hsv_green
def send_image_to_screen(cvimage):


    bridge = CvBridge()

    msg = bridge.cv2_to_imgmsg(cvimage, encoding="passthrough")
    
    pub = rospy.Publisher('/robot/xdisplay', Image,  queue_size=10)
    
    pub.publish(msg)
    # Sleep to allow for image to be published.
    rospy.sleep(1)

def Getmask(hsv, lower, upper):
    lower_color = np.array(lower, dtype=np.uint8)
    upper_color = np.array(upper, dtype=np.uint8)
    
    mask = cv2.inRange(hsv, lower_color, upper_color)
    
    return mask

def GetmaskForBlue(hsv):
    lower_blue = np.array([165,50,200], dtype=np.uint8)
    upper_blue = np.array([175,255,255], dtype=np.uint8)
    
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    
    return mask_blue

def GetmaskForRed(hsv):
    lower_red = np.array([165,50,200], dtype=np.uint8)
    upper_red = np.array([175,255,255], dtype=np.uint8)
    
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    
    
    return mask_red

#[30,100,50], [70,255,255]
def GetmaskForGreen(hsv):
    lower_green = np.array([30,100,50], dtype=np.uint8)
    upper_green = np.array([70,255,255], dtype=np.uint8)
    
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    
    return mask_green

def findContour(frame, mask):
    contours1,hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
        
    if not contours1:
        return None
    
    bpoints1 = []
    bpoints2 = []
        
    for cnt in contours1:
        if(len(cnt) > 20):
            # Draw bounding rectangles
            x,y,w,h = cv2.boundingRect(cnt)
         
                #print bound_rect
                # for more details about cv.BoundingRect,see documentation
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
            
        frame = cv2.rectangle(frame,tuple(pt1),tuple(pt2),(0,255,0),1)
        

            # HORIZON COORDINATE OF THE LARGEST RECTANGLE
        centroidu = (pt1[0]+pt2[0])/2
            # VERTICAL COORDINATE OF THE LARGEST RECTANGLE
        centroidv = (pt1[1]+pt2[1])/2
            #cv.Circle(cvimage,(centroidu,centroidv),5,0,-1)

           
            #frame = cv2.circle(frame,(centroidu,centroidv),(0,255,0),2)
        frame = cv2.circle(frame,(centroidu,centroidv),5,(0,255,0),-1)
        return [centroidu, centroidv]



def publish(color_location):
   
    pub = rospy.Publisher('obj_pos', String, queue_size=10)
    msg = ""
    for color in color_location:
        point = color_location[color]
        #(x0, y0, z0) 
        msg = msg + ";" + color + ":" + str(point[0]) + "," + str(point[1]) + "," + str(point[2])
        
    if not rospy.is_shutdown():
       
        print("Sending point command:", msg[1:])
        pub.publish(msg[1:])
        time.sleep(2)
    


def main():
    #http://hslpicker.com/#570000
    #dark green in the student unio"
    #RGB: R:0 G:102 B:51
    #darkgreen: 75,255,102
    
    #dark red:
    #RGB: 87 0 0
    #[  0 255  87]
    print 'Initializing node...'
    
    rospy.init_node('rsdk_xdisplay_image', anonymous=True)
    baxter.enable()
    boundaries = {'blue block':([0,150,50], [10,255,255]),
                 #'pink block' : ([165,50,200], [175,255,255]),
                 #'darkgreen block': ([65,255,102], [85,255,102]),
                 'green block': ([30,100,50], [80,255,255]),
                 'red block':([0,150,50], [10,255,255])
                 #[30,100,50], [70,255,255]
                 #'darkred block':([0, 255, 87], [10, 255, 87]),
                 #'yellow block' :([20,100,100], [30,255,255]),
                 #'orange block':([5,100,100], [15,255,255]),
                 
#                 'green':([25, 146, 190], [62, 174, 250])
                }
#     boundaries = {'blue card':([40,100,50], [80,255,255]),
#                  'red card' : ([0,50,50], [10,255,255]),
# #                  'red2': ([0,50,50], [10, 100, 100])
# #                 'yellow':([25, 146, 190], [62, 174, 250]),
# #                 'green':([25, 146, 190], [62, 174, 250])
#                 }
    
    while(1):
        # Take each frame
        #_, frame = cap.read()
        color_location = {}
        image = baxter.getImageFromRightHandCamera()
        frame = baxter.getLastCvImage()
        #send_image_to_screen(frame)
    
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        for color_name in boundaries:
            (lower, upper) = boundaries[color_name]
            mask = Getmask(hsv, lower, upper)
            
                
            p = findContour(frame, mask)
            if(p != None):
                color_location.update({color_name : (p[0], p[1], 0)})
        
        if(color_location):
            publish(color_location)    




if __name__ == '__main__':
   
#     test()
    main()
    #green R:0 G:102 B:51
#     green = np.uint8([[[0,0,87 ]]])
#     hsv_green = cv2.cvtColor(green,cv2.COLOR_BGR2HSV)
#     print hsv_green

 
 
 #darkgreen  60 255 100
 #light green 41 143 107       
