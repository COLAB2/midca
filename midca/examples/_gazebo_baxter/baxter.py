#!/usr/bin/env python
'''
Created on Jun 16, 2015

@author: baxter
'''
import rospy
import baxter_interface
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from sensor_msgs.msg import Image
#from baxter_srv.srv import ImageSrv, ImageSrvResponse
import numpy as np
import  cv2
from cv_bridge import CvBridge, CvBridgeError

class Baxter:
    def __init__(self):
        pass
    
    
 
    
    
#     def inverseKinematics(self, limb, point, orientation):
#         #ns = "/sdk/robot/limb/" + limb + "/solve_ik_position"
#         ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
#         rospy.wait_for_service(ns)
#         iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
#         ikreq = SolvePositionIKRequest()
#         hdr = Header(stamp=rospy.Time.now(), frame_id='base')
#         pose = PoseStamped(
#                     header=hdr,
#                     pose=Pose(
#                     position=Point(
#                         x=point[0],
#                         y=point[1],
#                         z=point[2],
#                     ),
#                     orientation=Quaternion(
#                         x=orientation[0],
#                         y=orientation[1],
#                         z=orientation[2],
#                         w=orientation[3],
#                     )
#                 )
#             );
#         ikreq.pose_stamp.append(pose)
#         try:
#             resp = iksvc(ikreq)
#         except rospy.ServiceException,e :
#             rospy.loginfo("Service call failed: %s" % (e,))
#         if (resp.isValid[0]):
#             #print("SUCCESS - Valid Joint Solution Found:")
#             # Format solution into Limb API-compatible dictionary
#             limb_joints = dict(zip(resp.joints[0].names, resp.joints[0].angles))
#             #print limb_joints
#             return limb_joints
#         else:
#             print("INVALID POSE - No Valid Joint Solution Found.")
#             return None
 # Gripper Methods:
    
    def calibrateLeftGripper(self):
        self.leftGripper.calibrate()
        
    def closeLeftGripper(self):
        self.leftGripper.close()
        
    def openLeftGripper(self):
        self.leftGripper.open()
    
    def inverseKinematics(self, limb, point, orientation):
        #rospy.init_node("rsdk_ik_service_client")
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    
    
        poses = {
            'left': PoseStamped(
                header=hdr,
                pose=Pose(
                    position=Point(
#                         x=0.657579481614,
#                         y=0.851981417433,
#                         z=0.0388352386502,
                        x=point.x,
                        y=point.y,
                        z=point.z,
                    ),
                    orientation=Quaternion(
                        x=orientation[0],
                        y=orientation[1],
                        z=orientation[2],
                        w=orientation[3],
                    ),
                ),
            ),
            'right': PoseStamped(
                header=hdr,
                pose=Pose(
                    position=Point(
                        x=0.656982770038,
                        y=-0.852598021641,
                        z=0.0388609422173,
                    ),
                    orientation=Quaternion(
                        x=0.367048116303,
                        y=0.885911751787,
                        z=-0.108908281936,
                        w=0.261868353356,
                    ),
                ),
            ),
        }
         
        ikreq.pose_stamp.append(poses[limb])
    
    
        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 1
    
    
        if (resp.isValid[0]):
            print("SUCCESS - Valid Joint Solution Found:")
                # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            
            return limb_joints
        else:
            print("INVALID POSE - No Valid Joint Solution Found.")
         
        return 0
        
            
    def ik(self, point, orientation):
        
        angles = self.inverseKinematics('left', point, orientation)
        if not angles:
            return False
        else:
            return True
            
        
    def moveLeftArm(self, point, orientation):
        angles = self.inverseKinematics('left', point, orientation)
        if not angles:
            print 'none'
            return None
        else: 
            print angles
        
        
        print 'I am here!!!!!' 
        print angles
            
        self.leftArm.move_to_joint_positions(angles) # 15 secs timeout default.

    def image_callback(self,ros_image):
	try:
	 frame = self.bridge.imgmsg_to_cv2(ros_image,"bgr8")
	except CvBridgeError,e:
	 print e
	self.last_cvimage = frame
	imgarray = np.array(frame,dtype=np.uint8)
	return imgarray
    
    
    def enable(self):
        print "start"
        self.robotEnable = baxter_interface.RobotEnable()
        self.robotEnable.enable()
        self.leftArm = baxter_interface.Limb('left')
        self.rightArm = baxter_interface.Limb('right')
        # Waits for the image service of right hand camera to become available.
        #rospy.wait_for_service('last_image')
        #self.rightHandCamera = rospy.ServiceProxy('last_image', ImageSrv)
	self.bridge = CvBridge()
	camera_name = "right_hand_camera"
        self.rightHandCamera = rospy.Subscriber('/cameras/' + camera_name + "/image", Image,self.image_callback)
        self.leftGripper = baxter_interface.Gripper('left')

    def initiate(self):
        self.leftArm = baxter_interface.Limb('left')
        self.rightArm = baxter_interface.Limb('right')
        self.leftGripper = baxter_interface.Gripper('left')
        
        
    def getLeftArmPosition(self):
        position = self.leftArm.endpoint_pose()
        x = position['position'].x
        y = position['position'].y
        z = position['position'].z
        return [x,y,z]
    
    def getLeftArmOrientation(self):
        orientation = self.leftArm.endpoint_pose()
        x = orientation['orientation'].x
        y = orientation['orientation'].y
        z = orientation['orientation'].z
        w = orientation['orientation'].w
        return [x,y,z,w]
    
    def getImageFromRightHandCamera(self):
        """
        This method returns a numpy array of the image that was captured from
        Baxter's right hand camera.
        
        """
        #request = self.rightHandCamera()
	self.bridge = CvBridge()
	camera_name = "right_hand_camera"
        cvimage_array = rospy.Subscriber('/cameras/' + camera_name + "/image", Image,self.image_callback)
        #cvimage = bridge.imgmsg_to_cv(imgmsg,'bgr8')
        #msg = cv_bridge.CvBridge().cv2_to_imgmsg(frame, encoding="bgr8")
        #cvimage = bridge.imgmsg_to_cv2(imgmsg, "bgr8")
        #cvimage = bridge.imgmsg_to_cv()
        # Save the last image captured.
        #self.last_cvimage = cvimage
        #cvimage_array = np.asarray(cvimage, dtype=np.uint8)
        return cvimage_array
        
    def getLastCvImage(self):
        return self.last_cvimage
    
    def getLastArrayImage(self):
        cvimage_array = np.asarray(self.last_cvimage, dtype=np.uint8)
        return cvimage_array
    
