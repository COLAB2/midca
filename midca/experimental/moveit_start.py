import subprocess
import signal
import sys

processes = []

def signal_handler(signal, frame):
	for process in processes:
		try:
			process.kill()
		except OSError:
			pass 
	subprocess.Popen(["rosrun", "baxter_tools", "enable_robot.py", "-d"])   
	sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

processes.append(subprocess.Popen(["rosrun", "baxter_tools", "enable_robot.py", "-e"]))
processes[0].wait()
processes.append(subprocess.Popen(["rosrun", "baxter_interface", "joint_trajectory_action_server.py"]))
processes.append(subprocess.Popen(["rosrun", "baxter_interface", "gripper_action_server.py"]))
processes.append(subprocess.Popen(["roslaunch", "baxter_moveit_config", "move_group.launch"]))
processes.append(subprocess.Popen(["roslaunch", "baxter_moveit_config", "planning_context.launch"]))
'''
for process in processes:
	try:
		process.kill()  
	except OSError:
		pass
subprocess.Popen(["rosrun", "baxter_tools", "enable_robot.py", "-d"])  
'''

