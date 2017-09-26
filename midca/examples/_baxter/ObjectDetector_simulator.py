from __future__ import print_function
import time
import rospy
from geometry_msgs.msg import Point, PointStamped

import numpy as np
import cv2
    
def colorDetector():
    cap = cv2.VideoCapture(0)

    while(1):

        # Take each frame
        _, frame = cap.read()
    
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
        # define range of blue color in HSV
        lower_red = np.array([165,50,200])
        upper_red = np.array([175,200,255])
    
        # Threshold the HSV image to get only blue colors
        thresh = cv2.inRange(hsv, lower_red, upper_red)
    
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame,frame, mask= thresh)
        
        
        image, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
        if(len(contours) > 0):
            location_buffer = len(contours)
            print(location_buffer)
            return location_buffer
        else:
            location_buffer = 0
            
        cv2.imshow('frame',frame)
        cv2.imshow('mask',thresh)
        cv2.imshow('res',res)
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            return 0

    cv2.destroyAllWindows()

def send_msgs(i):
    rospy.init_node('baxter_pointing_test')
    pub = rospy.Publisher('quad_pos', PointStamped, queue_size=10)
    points = [Point(x = 5.0, y = 0.0, z = 0.0),
              Point(x = 8.0, y = 7.0, z = 2.0),
              Point(x = 7.0, y = -4.0, z = -0.5), Point(x = -1, y = 1, z = 0)]
    n = 0
    if i > 0:
        n += 1
        time.sleep(2)
        p = points[n % 4]
        print("Sending point command:", p)
        pub.publish(PointStamped(point = p))

if __name__ == "__main__":
    i = colorDetector()
    send_msgs(i)