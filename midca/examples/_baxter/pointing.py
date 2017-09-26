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

limb = None
feedbackPub = None

def point_joint_angles(target):
    '''
    target is a numpy array or python list containing the 3d location of the thing to be pointed at, using robot-centered coordinates.
    '''
    
    #Center on location of the first point of rotation of the right arm
    base_joint_pos = np.array([0.07316, -0.26733, 0.31])
    target = target - base_joint_pos
    
    #set fixed angles to 0
    angles = {'right_e0': 0.0, 'right_e1': 0.0, 'right_w0': 0.0, 
    'right_w1': 0.0, 'right_w2': 0.0}
    
    #calculate variable angles
    angles['right_s0']=math.atan2(target[0], -target[1]) - np.pi/4
    angles['right_s1']=-math.atan2(target[2], np.linalg.norm(target[:2]))
    
    print("pointing: angles for joint right s0:", angles['right_s0'])
    print("pointing: angles for joint right s1:", angles['right_s1'])
    
    return angles

def check_range(target):
    '''
    target is a numpy array or python list containing the 3d location of the thing to be pointed at, using robot-centered coordinates.
    '''
    
    #Center on location of the first point of rotation of the right arm
    base_joint_pos = np.array([0.07316, -0.26733, 0.31])
    targetnew = target - base_joint_pos
    
    #calculate variable angles
    right_s0=math.atan2(targetnew[0], -targetnew[1]) - np.pi/4
    right_s1=-math.atan2(targetnew[2], np.linalg.norm(targetnew[:2]))

    return right_s0 < 1.55 and right_s0 > -1.6 and right_s1 < 1 and right_s1 > -1.35

def midca_feedback(**kwargs):
    msg = rosrun.dict_as_msg(kwargs)
    feedbackPub.publish(msg)

def point_callback(data, cmd_id = 'point_def_id'):
    if data.x == 0 and data.y == 0 and data.z == 0:
        rospy.loginfo("Point: got a stop command (all zeros)")
    elif check_range([data.x, data.y, data.z]):
        rospy.loginfo("Point: setting target to" + str(data))
        limb.move_to_joint_positions(point_joint_angles([data.x, data.y, 
        data.z]), threshold = 0.05)
        midca_feedback(cmd_id = cmd_id, code = asynch.COMPLETE)
        #limb.move_to_joint_positions(angles) #blocking
    else:
        rospy.loginfo("Point: target out of pointing range " + str(data))
        midca_feedback(cmd_id = cmd_id, code = asynch.FAILED)

def fake_point(data):
	print("pretending to point")
	msg = data.data
	d = rosrun.msg_as_dict(msg)
	time.sleep(2)
	print("sending feedback")
	midca_feedback(cmd_id = d['cmd_id'], code = asynch.COMPLETE)

def msg_callback(data):
    print("got msg:", data)
    msg = data.data
    d = rosrun.msg_as_dict(msg)
    if not 'x' in d and 'y' in d and 'z' in d and 'cmd_id' in d:
        rospy.logerr("Msg received by pointing node, but it does not contain" +
        "keys 'x', 'y', 'z', 'cmd_id'. Msg will be ignored.")
        midca_feedback(cmd_id = d['cmd_id'], code = asynch.FAILED)
    else:
        try:
            point = Point(x = int(d['x']), y = int(d['y']), z = int(d['z']))
        except NumberFormatException:
            rospy.logerr("Msg received by pointing nodes, but target coordinates" +
                         " are not well-formed. Msg will be ignored.")
            midca_feedback(cmd_id = d['cmd_id'], code = asynch.FAILED)
            return
        point_callback(point, d['cmd_id'])
    
def start_node(targetTopic, limbName = 'right'):
    rospy.init_node('baxter_point')
    rospy.loginfo("Reading point commands from topic " + targetTopic)
    #rospy.Subscriber(targetTopic, String, fake_point) 
    rospy.Subscriber(targetTopic, String, msg_callback)    
    global limb, feedbackPub
    feedbackPub = rospy.Publisher(rosrun.FEEDBACK_TOPIC, String, queue_size = 10)
    limb = baxter_interface.Limb(limbName)
    rospy.spin()

def test_angle_finder():
    rospy.init_node('baxter_point')
    points = [Point(x = 5.0, y = 0.0, z = 0.0),
              Point(x = 8.0, y = 3.0, z = 2.0),
              Point(x = 5.0, y = -1.0, z = -3.0)]
    n = 0
    while n < 20:
        n += 1
        time.sleep(1)
        p = points[n % 3]
        print(point_joint_angles([p.x, p.y, p.z]))
        point_callback(p)
        

if __name__ == '__main__':
    #test_angle_finder()
    #sys.exit()
    if len(sys.argv) > 2:
        raise Exception("Usage: 1 optional argument giving the topic on which commands are broadcast.")
    elif len(sys.argv) == 2:
        topic = sys.argv[1]
    else:
        topic = asynch.POINT_TOPIC
    try:
        start_node(topic)
    except rospy.ROSInterruptException:
        pass
