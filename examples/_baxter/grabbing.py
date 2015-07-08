from __future__ import print_function

# Import the necessary Python modules
import sys
import math
import numpy as np
import time

# rospy - ROS Python API
import rospy

# baxter_interface - Baxter Python API
import baxter_interface

#Message carrying location of target
from geometry_msgs.msg import Point
from std_msgs.msg import String
from MIDCA import rosrun
from MIDCA.modules._plan.asynch import asynch
from baxter import *

limb = None
feedbackPub = None
baxter = Baxter() # The object that controls Baxter. Defined in baxter.py.

def midca_feedback(**kwargs):
    msg = rosrun.dict_as_msg(kwargs)
    feedbackPub.publish(msg)

def move_left_arm_to(point):
    baxter.enable()
    origin = baxter.getLeftArmPosition()
    

    print('point =', point)
    baxter.moveLeftArm(point, baxter.getLeftArmOrientation())
    


def point_callback(data, cmd_id = 'point_def_id'):
    print("I M HERE******* ", data)
    
    if data.x == 0 and data.y == 0 and data.z == 0:
        rospy.loginfo("Point: got a stop command (all zeros)")
    else:
        rospy.loginfo("Point: setting target to" + str(data))
       
        move_left_arm_to(data)
        
        midca_feedback(cmd_id = cmd_id, code = asynch.COMPLETE)
        #limb.move_to_joint_positions(angles) #blocking
#     else:
#         rospy.loginfo("Point: target out of pointing range " + str(data))
#         midca_feedback(cmd_id = cmd_id, code = asynch.FAILED)


def msg_callback(data):
    print("got msg:", data)
    msg = data.data
    d = rosrun.msg_as_dict(msg)
    
    if not 'x' in d and 'y' in d and 'z' in d and 'cmd_id' in d:
        rospy.logerr("Msg received by grabbing node, but it does not contain" +
        "keys 'x', 'y', 'z', 'cmd_id'. Msg will be ignored.")
        midca_feedback(cmd_id = d['cmd_id'], code = asynch.FAILED)
    else:
        try:
            
            
            point = Point(x = d['x'], y = d['y'], z = d['z'])
            
        except NumberFormatException:
            rospy.logerr("Msg received by grabbing nodes, but target coordinates" +
                         " are not well-formed. Msg will be ignored.")
            midca_feedback(cmd_id = d['cmd_id'], code = asynch.FAILED)
            return
        point_callback(point, d['cmd_id'])
    
def start_node(targetTopic, limbName = 'left'):
    rospy.init_node('baxter_grabbing')
    rospy.loginfo("Reading point commands from topic " + targetTopic)
    #rospy.Subscriber(targetTopic, String, fake_point) 
    rospy.Subscriber(targetTopic, String, msg_callback)    
   
    
    global limb, feedbackPub
    feedbackPub = rospy.Publisher(rosrun.FEEDBACK_TOPIC, String, queue_size = 10)
    limb = baxter_interface.Limb(limbName)
    rospy.spin()


if __name__ == '__main__':
    #test_angle_finder()
    #sys.exit()
    if len(sys.argv) > 2:
        raise Exception("Usage: 1 optional argument giving the topic on which commands are broadcast.")
    elif len(sys.argv) == 2:
        topic = sys.argv[1]
    else:
        topic = "loc_cmd"
    try:
        start_node(topic)
    except rospy.ROSInterruptException:
        pass
