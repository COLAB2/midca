#!/usr/bin/env python


import argparse

import sys

import os,inspect


import rospy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
    
)
from geometry_msgs.msg import (
    Pose,
    Point,
    
)
from std_msgs.msg import (
    Header,
    Empty,
)



import baxter_interface
import baxter_external_devices
from baxter_interface import CHECK_VERSION


def load_gazebo_models(table_pose=Pose(position=Point(x=1.0, y=0.0, z=0.00)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.6725, y= 0.1265, z=0.7825)),
                       block_reference_frame="world",
		       block_pose1=Pose(position=Point(x=0.7425, y= 0.0, z=0.7825)),
                       block_reference_frame1="world",
		       block_pose2=Pose(position=Point(x=0.6325, y= 0.05, z=0.7825)),
                       block_reference_frame2="world",
		       block_pose3=Pose(position=Point(x=0.5525, y= 0.2265, z=0.7825)),
                       block_reference_frame3="world",
		       block_pose4=Pose(position=Point(x=0.7925, y= 0.2265, z=0.7825)),
                       block_reference_frame4="world",
		       block_pose5=Pose(position=Point(x=0.7925, y= -0.12, z=0.7825)),
                       block_reference_frame5="world",
		       block_pose6=Pose(position=Point(x=0.5525, y= -0.12, z=0.7825)),
                       block_reference_frame6="world"):

    p1 = [block_pose3.position.x , block_pose3.position.y , block_pose3.position.z]
    p2 = [block_pose4.position.x , block_pose4.position.y , block_pose4.position.z]
    p3 = [block_pose5.position.x , block_pose5.position.y , block_pose5.position.z]
    p4 = [block_pose6.position.x , block_pose6.position.y , block_pose6.position.z]
    
    calibration(p1,p2,p3,p4)
    model_path = thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))+"/models/"

    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')
   
    block_xml = ''
    with open (model_path + "block/model_red.urdf", "r") as block_file:
        block_xml=block_file.read().replace('\n', '')

    block_xml1 = ''
    with open (model_path + "block/model_green.urdf", "r") as block_file:
        block_xml1=block_file.read().replace('\n', '')

    block_xml2 = ''
    with open (model_path + "block/model_blue.urdf", "r") as block_file:
        block_xml2=block_file.read().replace('\n', '')

    block_xml3 = ''
    with open (model_path + "paper/model.urdf", "r") as block_file:
        block_xml3=block_file.read().replace('\n', '')
   
	

    
   
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

   
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block_red", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block_green", block_xml1, "/",
                               block_pose1, block_reference_frame1)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block_blue", block_xml2, "/",
                               block_pose2, block_reference_frame2)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("paper_lb", block_xml3, "/",
                               block_pose3, block_reference_frame3)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("paper_lt", block_xml3, "/",
                               block_pose4, block_reference_frame4)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("paper_rt", block_xml3, "/",
                               block_pose5, block_reference_frame5)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("paper_rb", block_xml3, "/",
                               block_pose6, block_reference_frame6)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))
    
def calibration(p1,p2,p3,p4):
    thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) + "/../"
	    
    filename = thisDir + "/_baxter/calibration.txt"
    
    target = open(filename, 'w')

   
    target.truncate()
    
    target.write(repr(p1[0])+ " " + repr(p1[1]))
    target.write("\n")
    target.write(repr(p2[0])+ " " + repr(p2[1]))
    target.write("\n")
    target.write(repr(p3[0])+ " " + repr(p3[1]))
    target.write("\n")
    target.write(repr(p4[0])+ " " + repr(p4[1]))
    target.write("\n")
    
    
    target.close()
    

    

def delete_gazebo_models():
    
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")
        resp_delete = delete_model("block_red")
	resp_delete = delete_model("block_green")
	resp_delete = delete_model("block_blue")
	resp_delete = delete_model("paper_lb")
	resp_delete = delete_model("paper_lt")
	resp_delete = delete_model("paper_rt")
	resp_delete = delete_model("paper_rb")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))



def main():
    
   
    print("\nGetting models...")
    load_gazebo_models()
    rospy.init_node("model")
    #rs = baxter_interface.RobotEnable(CHECK_VERSION)
    #init_state = rs.state().enabled
     
   

    
    def clean_shutdown():
	delete_gazebo_models()
        print("\nExiting models...")
        #if not init_state:
        #    print("Disabling robot...")
        #    rs.disable()

    rospy.on_shutdown(clean_shutdown)

    #print("Enabling robot... ")
    #rs.enable()
    rospy.sleep(1)

	
    while not rospy.is_shutdown():
     i=1;

    if rospy.is_shutdown():
         rospy.signal_shutdown("Models finished.")   
    
    print("Done.")
    

if __name__ == '__main__':
    sys.exit(main())
