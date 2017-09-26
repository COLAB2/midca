from midca.experimental.baxter import baxter, ik_matt as ik
import rospy, baxter_interface

class BaxterWave:

	'''
	MIDCA module which causes connected Baxter robot to wave its right arm. Note: baxter must be turned on, and the baxter.sh file should be sourced to connect to baxter.
	'''
	def __init__(self, leftArmPositions, rightArmPositions):
		self.leftPositions = leftArmPositions
		self.rightPositions = rightArmPositions
	
	def init(self, world, mem):
		self.mem = mem
	
	def run(self, cycle, verbose = 2):
		rospy.init_node('MIDCA_wave')
		baxter.enable_robot(nodeEnabled = True)
		right = baxter_interface.Limb('right')
		left = baxter_interface.Limb('left')
		print "calculating inverse kinematics solutions"
		solutionsRight = []
		solutionsLeft = []
		for pos in self.rightPositions:
			solution = ik.getSolution(pos, right.joint_angles(), 'right', verbose = False)
			if solution:
				solutionsRight.append(solution)
			else:	
				if verbose >= 1:
					print "unable to generate solution for ", str(pos), ", Skipping"
		for pos in self.leftPositions:
			solution = ik.getSolution(pos, left.joint_angles(), 'left', verbose = False)
			if solution:
				solutionsLeft.append(solution)
			else:	
				if verbose >= 1:
					print "unable to generate solution for ", str(pos), ", Skipping"
		for i in range(max(len(solutionsRight), len(solutionsLeft))):
			try:
				right.move_to_joint_positions(solutionsRight[i])
			except IndexError:
				pass
			try:
				left.move_to_joint_positions(solutionsLeft[i])
			except IndexError:
				pass

def test():
	wave = BaxterWave([], [(0.4, -0.2, 0.2), (0.5, -0.2, 0.4), (0.4, -0.2, 0.2)])
	wave.run(1)

if __name__ == "__main__":
	test()
		
