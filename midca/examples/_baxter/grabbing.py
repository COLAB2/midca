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
from midca import rosrun
from midca.modules._plan.asynch import asynch
from midca.examples.baxter import *

limb = None
feedbackPub = None
baxter = Baxter() # The object that controls Baxter. Defined in baxter.py.

def midca_feedback(**kwargs):
    msg = rosrun.dict_as_msg(kwargs)
    feedbackPub.publish(msg)

def MoveArm(point):
    
    #o1 = [0.9912059087997489, 0.12898810957822202, -0.016456430713070586, 0.02453772271597873]
    #o1 = [-0.04504873726153096, 0.9983659324956183, 0.020392051958720396, -0.028639837992011533]
    #o1 = [-0.018892183334939267, 0.9997389606745517, 0.002440171629185176, -0.012615129037248417]
    o1 = [-0.019402852593377543, 0.9997363871414516, 0.0032541057059402015, 0.011836229663689808]
    print('point =', point)
    
    #[0.9858036680068526, 0.15518960733015813, -0.06358533543017306, 0.008013678255261263]
    return baxter.moveLeftArm(point, o1)
    
    
    
def Grasp():
    baxter.closeLeftGripper()
    return True
def release():
    baxter.openLeftGripper()
    return True

def point_2_callback(point, cmd_id = 'point_2_def_id'):
    
    if point.x == 0 and point.y == 0 and point.z == 0:
        rospy.loginfo("Point: got a stop command (all zeros)")
    else:
        rospy.loginfo("Point: setting target to" + str(point))
        
        if(MoveArm(point)== None):
            rospy.loginfo("Point: target out of pointing range " + str(point))
            midca_feedback(cmd_id = cmd_id, code = asynch.FAILED)
        
        else:
            #baxter.openLeftGripper()
            midca_feedback(cmd_id = cmd_id, code = asynch.COMPLETE)
            
        #limb.move_to_joint_positions(angles) #blocking
#     else:
#         rospy.loginfo("Point: target out of pointing range " + str(data))
#         midca_feedback(cmd_id = cmd_id, code = asynch.FAILED)

def msg_callback(point, cmd_id = 'grasp_id'):
    
        Grasp()
        
        midca_feedback(cmd_id = cmd_id, code = asynch.COMPLETE)
        #limb.move_to_joint_positions(angles) #blocking
#     else:
#         rospy.loginfo("Point: target out of pointing range " + str(data))
#         midca_feedback(cmd_id = cmd_id, code = asynch.FAILED)

def release_callback(point, cmd_id = 'r_id'):
        print("yeah got it")
        release()
        
        midca_feedback(cmd_id = cmd_id, code = asynch.COMPLETE)

def point_callback(point, cmd_id = 'point_def_id'):
    
    if point.x == 0 and point.y == 0 and point.z == 0:
        rospy.loginfo("Point: got a stop command (all zeros)")
    else:
        rospy.loginfo("Point: setting target to" + str(point))

        if(MoveArm(point)== None):
            rospy.loginfo("Point: target out of pointing range " + str(point))
            midca_feedback(cmd_id = cmd_id, code = asynch.FAILED)
        
        else:
            midca_feedback(cmd_id = cmd_id, code = asynch.COMPLETE)
        #limb.move_to_joint_positions(angles) #blocking
#     else:
#         rospy.loginfo("Point: target out of pointing range " + str(data))
#         midca_feedback(cmd_id = cmd_id, code = asynch.FAILED)
def reaching_callback(data):
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
        
def raising_callback(data):
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
            
            
        except Exception:
            rospy.logerr("Msg received by grabbing nodes, but target coordinates" +
                         " are not well-formed. Msg will be ignored.")
            midca_feedback(cmd_id = d['cmd_id'], code = asynch.FAILED)
            return
        point_2_callback(point, d['cmd_id'])


def releasing_callback(data):
    print("got msg:", data)
    msg = data.data
    d = rosrun.msg_as_dict(msg)
    
    release_callback(msg, d['cmd_id'])
   

def grabbing_callback(data):
    print("got msg:", data)
    msg = data.data
    d = rosrun.msg_as_dict(msg)
    
    msg_callback(msg, d['cmd_id'])
    

    
def start_node(targetTopic, limbName = 'left'):
    rospy.init_node('baxter_grabbing')
    print("start node")
    rospy.loginfo("Reading point commands from topic " + targetTopic)
    #rospy.Subscriber(targetTopic, String, fake_point) 
    rospy.Subscriber("loc_cmd", String, reaching_callback)    
    rospy.Subscriber("grabbing_cmd", String, grabbing_callback)
    rospy.Subscriber("raise_cmd", String, raising_callback)
    rospy.Subscriber("release_cmd", String, releasing_callback)
    baxter.initiate()
    
    #origin = baxter.getLeftArmPosition()
    
    
    global feedbackPub
    feedbackPub = rospy.Publisher(rosrun.FEEDBACK_TOPIC, String, queue_size = 10)
    #limb = baxter_interface.Limb(limbName)
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
