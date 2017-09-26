
import rospy, baxter_interface
import math

from baxter_pykdl import baxter_kinematics

def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2)

class Range:
	
    def __init__(self, min, max):
    	self.min = min
    	self.max = max
	
    def __contains__(self, val):
    	return val >= self.min and val <= self.max
    
    def midpoint(self):
    	return (self.min + self.max) / 2
    
    def nearCenter(self, val):
    	try:
    		return 1.0001 - math.sqrt(abs(val - self.midpoint()) / (self.max - self.midpoint()))
    	except ValueError:
    		#not in range
    		return 0

angleRanges = {'right_s0': Range(-2, .5), 'right_s1': Range(-1.35, 0.5), 'right_w0': Range(-2.5, 2.5), 'right_w1': Range(-1, 1.5), 'right_w2': Range(-2.5, 2.5), 'right_e0': Range(-2.5, 2.5), 'right_e1': Range(0.3, 2.1), 'left_s0': Range(-2, .5), 'left_s1': Range(-1.35, 0.5), 'left_w0': Range(-2.5, 2.5), 'left_w1': Range(-1, 1.5), 'left_w2': Range(-2.5, 2.5), 'left_e0': Range(-2.5, 2.5), 'left_e1': Range(0.3, 2.1)}


def getSolution(target, startAngles, limb, threshold = 0.01, delta = 0.005, verbose = True):
    kin = baxter_kinematics(limb)
    curPosition = kin.forward_position_kinematics(startAngles)
    if verbose:	
    	print "Starting IK engine - starting pose:", curPosition
    curAngles = {name: angleRanges[name].midpoint() for name in startAngles.keys()}
    print curAngles
    curPosition = kin.forward_position_kinematics(curAngles)
    n = 0
    movesUp = {name: 0 for name in curAngles.keys()}
    movesDown = {name: 0 for name in curAngles.keys()}
    while distance(curPosition, target) > threshold:
    	n += 1
    	if n > 10000:
    		if verbose:
    			print "Reached 10000 steps. Position:", curPosition
    			print "up:", movesUp
    			print "down", movesDown
    		return None
    	d = distance(curPosition, target)
    	deltaD = []
    	for name in curAngles.keys():
    		#calculate distance from target if joint is rotated "up" or "down" by delta radians. Ignore results which exceed min/max joint angles, or which are farther than current distance.
    		curAngles[name] += delta
    		if curAngles[name] in angleRanges[name]:
    			dUp = distance(kin.forward_position_kinematics(curAngles), target)
    		else:
    			dUp = float("inf")
    		curAngles[name] -= 2 * delta
    		if curAngles[name] in angleRanges[name]:
    			dDown = distance(kin.forward_position_kinematics(curAngles), target)
    		else:
    			dDown = float("inf")
    		curAngles[name] += delta
    		if dUp < d and dUp < dDown:
    			deltaD.append((dUp - d, name, True))
    		elif dDown < d:
    			deltaD.append((dDown - d, name, False))
    	if not deltaD:
    		if verbose:
    			print "No useful moves found at position:", curPosition
    		return None
    	deltaD.sort(key = lambda x: x[0])
    	if deltaD[0][2]:
    		curAngles[deltaD[0][1]] += delta
    		movesUp[deltaD[0][1]] += 1
    	else:
    		curAngles[deltaD[0][1]] -= delta
    		movesDown[deltaD[0][1]] += 1
    	curPosition = kin.forward_position_kinematics(curAngles)
    print "found solution in", n, "steps."
    return curAngles

from heapq import heappush, heappop

class Hashable:
	
    def __init__(self, d):
    	l = [d[key] for key in sorted(list(d.keys()))]
    	self.tuple = tuple(l)
    
    def __hash__(self):
    	return hash(self.tuple)
    
    def __eq__(self, other):
    	try:
    		return self.tuple == other.tuple
    	except AttributeError:
    		return False
   
def getSolutionAStar(target, startAngles, threshold = 0.01, delta = 0.01, verbose = True):
    kin = baxter_kinematics('right')
    stateQ = []
    visited = set()
    curPosition = kin.forward_position_kinematics(startAngles)
    if verbose:	
    	print "Starting IK engine - starting pose:", curPosition
    curAngles = {name: angleRanges[name].midpoint() for name in startAngles.keys()}
    curPosition = kin.forward_position_kinematics(curAngles)
    heappush(stateQ, (distance(curPosition, target), dict(curAngles)))
    visited.add(Hashable(curAngles))
    n = 0
    movesUp = {name: 0 for name in curAngles.keys()}
    movesDown = {name: 0 for name in curAngles.keys()}
    while stateQ:
    	n += 1
    	d, curAngles = heappop(stateQ)
    	if d < threshold:
    		print "found solution in", n, "steps."
    		return curAngles
    	if n > 10000:
    		if verbose:
    			print "Reached 10000 steps. Position:", kin.forward_position_kinematics(curAngles)
    			print "Angles"
    		return None
    	deltaD = []
    	for name in curAngles.keys():
    		#calculate distance from target if joint is rotated "up" or "down" by delta radians. Ignore results which exceed min/max joint angles
    		curAngles[name] += delta
    		if Hashable(curAngles) not in visited and curAngles[name] in angleRanges[name]:
    			dUp = distance(kin.forward_position_kinematics(curAngles), target)
    			visited.add(Hashable(curAngles))
    			heappush(stateQ, (dUp, dict(curAngles)))
    		curAngles[name] -= 2 * delta
    		if Hashable(curAngles) not in visited and curAngles[name] in angleRanges[name]:
    			dDown = distance(kin.forward_position_kinematics(curAngles), target)
    			visited.add(Hashable(curAngles))
    			heappush(stateQ, (dDown, dict(curAngles)))
    		curAngles[name] += delta
    if verbose:
    	print "No useful moves found at step", n
    return None

def test():
	#sol = getSolutionAStar([0.55, -0.4, -0.6], {'right_s0': 0, 'right_s1': 0, 'right_e0': 0, 'right_e1': 0, 'right_w0': 0, 'right_w1': 0, 'right_w2': 0}, threshold = 0.1)
	#print sol
	rospy.init_node('baxter_kinematics')
	import baxter
	baxter.enable_robot(nodeEnabled = True)
	sol = getSolution([0.5, 0, -0.2], {'right_s0': 0, 'right_s1': 0, 'right_e0': 0, 'right_e1': 0, 'right_w0': 0, 'right_w1': 0, 'right_w2': 0}, 'right', threshold = 0.01)
	print sol
	rospy.init_node('baxter_kinematics')
	kin = baxter_kinematics('right')
	pos = kin.forward_position_kinematics(sol, var = True)
	print pos
	right = baxter_interface.Limb('right')
	right.move_to_joint_positions(sol)
	print right.endpoint_pose()['position']
	print right.joint_angles()
    	
if __name__ == "__main__":
	test()
	
