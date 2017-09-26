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



def publish(msg, topic):
   
    pub = rospy.Publisher(topic, String, queue_size=10)

    if not rospy.is_shutdown():
       
        print("Sending point command:", "monitor fires")
        pub.publish(msg)
        time.sleep(2)
    

       
    
def monitor_clear_block(block_name='red block', topic='clear_block'):
   print "monitoring... "
                


if __name__ == '__main__':

    main()
