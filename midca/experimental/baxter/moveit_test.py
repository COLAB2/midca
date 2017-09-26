import subprocess, sys, time
from moveit_commander import MoveGroupCommander
import moveit_msgs.msg
import geometry_msgs.msg

class BaxterWave:
	
	def __init__(self, leftArmPositions, rightArmPositions):
		self.leftPositions = leftArmPositions
		self.rightPositions = rightArmPositions
		
	def run(self, cycle, verbose = 2):

		#enable robot if not already done
		enableProcess = subprocess.Popen(["rosrun", "baxter_tools", 
		"enable_robot.py", "-e"])
		enableProcess.wait()
		
		#start moveit servers. Since we do not know how long it will take them 
		#to start, pause for 15 seconds
		jointServer = subprocess.Popen(["rosrun", "baxter_interface", 
		"joint_trajectory_action_server.py"])
		planServer = subprocess.Popen(["roslaunch", "baxter_moveit_config", 
		"move_group.launch"])
		time.sleep(15)
		
		raw_input("press enter")
		
		try:
			#left = MoveGroupCommander("left_arm")
			right = MoveGroupCommander("right_arm")
			
			upRight = geometry_msgs.msg.Pose(position = geometry_msgs.msg.Point(
			x = 0.9667369015076861, y = -0.1190874231664102, z = -0.07489146157788866 ))
			
			#upLeft = geometry_msgs.msg.Pose(position = geometry_msgs.msg.Point(
			#x = 0.7, y = 0.35, z = 0.8))
			
			#downRight = geometry_msgs.msg.Pose(position = geometry_msgs.msg.Point(
			#x = 0.7, y = -0.45, z = 0.3))
			
			#downLeft = geometry_msgs.msg.Pose(position = geometry_msgs.msg.Point(
			#x = 0.7, y = 0.45, z = 0.3))
			
			right.set_pose_target(upRight)
			#left.set_pose_target(upLeft)
			right.go()
			#left.go()
			
			#right.set_pose_target(downRight)
			#left.set_pose_target(downLeft)
			
			right.go()
			#left.go()
			
		finally:
			#clean up - kill servers
			jointServer.kill()
			planServer.kill()
			
			sys.exit()
		
def test():
	wave = BaxterWave([], [(0.4, -0.2, 0.2), (0.5, -0.2, 0.4), (0.4, -0.2, 0.2)])
	wave.run(1)

if __name__ == "__main__":
	test()