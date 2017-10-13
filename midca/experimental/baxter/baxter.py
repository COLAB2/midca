import rospy

import baxter_interface
from baxter_interface import CHECK_VERSION

def enable_robot(nodeEnabled = False):
	if not nodeEnabled:
		rospy.init_node('rsdk_robot_enable')
	rs = baxter_interface.RobotEnable(CHECK_VERSION)
	try:
		rs.enable()
	except Exception, e:
		rospy.logerr(e.strerror)

def disable_robot(nodeEnabled = False):
	if not nodeEnabled:
		rospy.init_node('rsdk_robot_enable')
	rs = baxter_interface.RobotEnable(CHECK_VERSION)
	try:
		rs.disable()
	except Exception, e:
		rospy.logerr(e.strerror)

def stop_robot(nodeEnabled = False):
	if not nodeEnabled:
		rospy.init_node('rsdk_robot_enable')
	rs = baxter_interface.RobotEnable(CHECK_VERSION)
	try:
		rs.stop()
	except Exception, e:
		rospy.logerr(e.strerror)
