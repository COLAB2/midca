#!/usr/bin/env python
import baxter_interface
import rospy
from sensor_msgs.msg import Image


rospy.init_node("my_cam")
display_pub= rospy.Publisher('/robot/xdisplay',Image, queue_size=10)
def republish(msg):
        """
            Sends the camera image to baxter's display
        """             
        display_pub.publish(msg)

camera_name = "right_hand_camera"
sub = rospy.Subscriber('/cameras/' + camera_name + "/image", Image,republish,None,1)
rospy.spin()
