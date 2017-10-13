#!/usr/bin/env python



import argparse
import math
import random
import rospy

from std_msgs.msg import(UInt16,)
import baxter_interface
from baxter_interface import CHECK_VERSION

class CameraPosition(object):
    def __init__(self):
        self._limb = baxter_interface.limb.Limb("right")
        self._angles = self._limb.joint_angles()
	# open left gripper
	#self.leftGripper =baxter_interface.Gripper('left')
	#self.leftGripper.calibrate()
	#self.leftGripper.open()
	# open Right gripper
	self.rightGripper =baxter_interface.Gripper('right')
	self.rightGripper.calibrate()
	self.rightGripper.open()
        # set control params

        print("Getting robot state...")
        self._rs=baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state= self._rs.state().enabled
        print("Enabling robot..")
        self._rs.enable()

    def set_camera(self):
        """
        These are hardcoded to position the right camera over the table to view blocks
        """
        print("Moving Right Hand Camera to View Blocks...")
        self._angles['right_s0']=-0.100
        self._angles['right_s1']=-0.9676139
        self._angles['right_e0']=1.58794
        self._angles['right_e1']=0.85706
        self._angles['right_w0']=-0.59094
        self._angles['right_w1']=2.0780
        self._angles['right_w2']= -1.288329
        self._limb.move_to_joint_positions(self._angles)


def main():

    print("Initializing node... ")
    rospy.init_node("rsdk_position_right_camera")
    cameraPos=CameraPosition()
    cameraPos.set_camera()
    rospy.sleep(1)
    

if __name__ == '__main__':
    main()
