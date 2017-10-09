import cv2
import numpy as np
from _dbus_bindings import String
import math

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


def Getmask(hsv, lower, upper):
    lower_color = np.array(lower, dtype=np.uint8)
    upper_color = np.array(upper, dtype=np.uint8)
    
    mask = cv2.inRange(hsv, lower_color, upper_color)
    
    return mask

def GetmaskForBlue(hsv):
    lower_blue = np.array([100,150,0], dtype=np.uint8)
    upper_blue = np.array([140,255,255], dtype=np.uint8)
    
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
    a,contours1,hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
        
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



def publish(msg, topic):
   
    pub = rospy.Publisher(topic, String, queue_size=10)

    if not rospy.is_shutdown():
       
        print("Sending point command:", "monitor fires")
        pub.publish(msg)
        time.sleep(2)
    
block_name = "red block"
def monitor_clear_block(block_n='red block', topic='clear_block'):
    print 'call monitor'
#     rospy.init_node('clear_block_monitoring_____...')
    block_name = block_n
    handler.subscriber = rospy.Subscriber("obj_pos", String, m_callback)

def m_callback(data):
    print("got data!")
    strMsg = str(data.data).strip()
    color_locations = strMsg.split(";")
    color_location_dic = {}
    for msg in color_locations:
            
        color_location = msg.split(":");
            
        pointstr = color_location[1].split(",")
        p = Point(x = int(pointstr[0]), y = int(pointstr[1]), z = int(pointstr[2]))
        color_location_dic.update({color_location[0]: p})
            
    
    
    if len(color_location) >= 2:
            pos_green = 'table'
            pos_red = 'table'
            clear_green = 'clear'
            clear_red = 'clear'
            for obj in color_location.keys():
                if obj != block_name:
                    if math.fabs(color_location[block_name].x - color_location[obj].x) < 12:
                        if color_location[block_name].y> color_location[obj].y:
                             #print block_name + " is not clear"
                             return obj

def monitor_clear_block2(block_name='red block', topic='clear_block'):
    print 'monitoring clear...'
    
    #rospy.init_node('clear_block_monitor', anonymous=True)
    baxter.enable()
    boundaries = {'blue block':([90,120,60], [120,255,255]),
                 'green block': ([50,100,50], [80,255,255]),
                 'red block':([0,150,50], [10,255,255])
                }
   
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
        time.sleep(2)
        if len(color_location) == 3:
            pos_green = 'table'
            pos_red = 'table'
            pos_blue = 'table'
            clear_green = 'clear'
            clear_red = 'clear'
            clear_blue = 'clear'
            for obj in color_location.keys():
                if obj != block_name:
                    if math.fabs(color_location[block_name][0] - color_location[obj][0]) < 12:
                        if color_location[block_name][1] > color_location[obj][1]:
                             #print block_name + " is not clear"
                             return obj
#                         else:
#                             print block_name + "is clear"
                

def monitor_clear_block_asus(block_n='red block', topic='clear_block'):
    #rospy.init_node('clear_block_monitoring...')
    block_name = block_n
    while(1):
    	msg = rospy.wait_for_message(topic, String)
    	obj,loc = m_asus(msg,block_name)
    	if obj:
		return obj,loc	    
	  
def m_asus(data,block_name):

    #print("got data!")
    strMsg = str(data.data).strip()
    color_locations = strMsg.split(";")
    color_locations.pop() # remove last unwanted ;
    color_location_dic = {}
    for msg in color_locations:   
        color_location = msg.split(":")    
        pointstr = color_location[1].split(",")
        p = Point(x = float(pointstr[0]), y = float(pointstr[1]), z = float(pointstr[2]))
        color_location_dic.update({color_location[0]: p})


    if color_location_dic :
		for each_block in color_location_dic:
			pos = 'table'
			clear = 'clear'
			cmp_block = block_name
			if not cmp_block in color_location_dic:
				break
			if not each_block == cmp_block:
				# check stack condition
				x_difference = abs(color_location_dic[each_block].x - color_location_dic[cmp_block].x)
				y_difference = abs(color_location_dic[each_block].y - color_location_dic[cmp_block].y)
				z_difference = abs(color_location_dic[each_block].z - color_location_dic[cmp_block].z)
				if ( x_difference < 0.03 and y_difference < 0.01 and z_difference > 0.02 and z_difference < 0.06):
					if (color_location_dic[each_block].z < color_location_dic[cmp_block].z):
						clear = 'not clear'
					else:
						pos = cmp_block;
						block_pos = each_block
						return each_block,color_location_dic[each_block]
            





if __name__ == '__main__':

    main()
