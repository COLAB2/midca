import copy
from midca.modules._adist import ADistance, ChangeFinder, WindowPair, Interval
from midca.domains.nbeacons import nbeacons_util
import sys
'''
See class ADistanceAnomalyNoter
'''

ANOMALY_STATE_KEY = "anomaly state"
A_DIST_KEY = "A-Distance memory"

def get_pred_names(world):
	return sorted(world.predicates.keys())

def get_pred_counts(world):
	counts = {}
	for name in get_pred_names(world):
		counts[name] = 0
	for atom in world.atoms:
		counts[atom.predicate.name] += 1
	return counts

def get_count_vector(world):
	counts = get_pred_counts(world)
	vector = []
	for name in get_pred_names(world):
		vector.append(counts[name])
	return vector

class ADistTrace:
	
	def __init__(self, prednames, startTime = 1):
		self.prednames = prednames
		self.values = [] #tuples: value, theshold
		self.startTime = startTime
	
	def update(self, distobjs):
		stepvals = []
		for cf in distobjs:
			stepvals.append((cf.getDistances()[0], cf.windowPairs[0].alpha))
		self.values.append(stepvals)
	
	def sequences(self):
		sequences = []
		for name in self.prednames:
			sequences.append([])
		for step in self.values:
			for i in range(len(step)):
				sequences[i].append(step[i][0])
		return sequences
	
	def val_to_str(self, val, threshold):
		if val > threshold:
			return '\033[94m' + str(round(val, 2)) + '\033[0m'
		else:
			return str(round(val, 2))
	
	def short_str(self, length, threshold = None):
		s = "time"
		for name in [self.prednames[0]] + self.prednames[3:5]:
			s += "\t|\t" + name[:7]
		i = self.startTime
		if len(self.values) > length:
			startnum = length / 2
			endnum = length - startnum
			values = self.values[:startnum] + self.values[-endnum:]
		else:
			values = self.values
			startnum = len(values) + 1
			endnum = 0
		for line in values:
			s += "\n" + str(i)
			for val in [line[0]] + line[3:5]:
				if threshold:
					valstr = self.val_to_str(val[0], threshold)
				else:
					valstr = self.val_to_str(val[0], val[1])
				s += "\t|\t" + valstr
			if i == startnum:
				print "\n...\n..."
				i = len(self.values) - endnum
			i += 1
		return s
	
	def __str__(self):
		return self.short_str(100)

class ADistWrapper:
	'''
	This class is designed for simple domain change representations where values for predicate count changes are resticted to [-1, 0, 1].This should work with most classical planning domains since only one simple action is executed per step, but there are certainly plausible exceptions. To generalize this approach, one should generate some sample data (e.g. by taking observations for a while under normal operating conditions), then initialize the A-distance object using the addProportional() method in adist/ADistance.py.
	
	Also, currently only the simple two-window approach is supported.
	'''
	def __init__(self, threshold, windowsize = 10, start = 1):
		self.distobjs = []	#a-distance	objects, ordered by predicate
		self.threshold = threshold
		self.windowsize = windowsize
		self.start = start
	
	def init_adist(self, prednames):
		for name in prednames:
			distobj = ADistance.ADistance()
			cf = ChangeFinder.ChangeFinder(distobj)
			for i in [-1.5, -0.5, 0.5]:
				distobj.add(Interval.Interval(i, i + 1))
			cf.addWindowPair(WindowPair.WindowPair(self.windowsize, self.windowsize, self.threshold))
			distobj.init(cf)
			self.distobjs.append(cf)
	
	def init(self, world):
		self.prednames = get_pred_names(world)
		self.init_adist(self.prednames)
		self.lastPredCounts = get_count_vector(world)
		self.trace = ADistTrace(self.prednames, self.start)
	
	def update(self, world):
		newPredCounts = get_count_vector(world)
		countdifs = [new - old for new, old in zip(newPredCounts, self.lastPredCounts)]
		for i in range(len(countdifs)):
			self.distobjs[i].addData(countdifs[i])
		self.trace.update(self.distobjs)
		self.lastPredCounts = newPredCounts
	
	def __str__(self):
		return str(self.trace)

class AnomalyDetector:
	
	def __init__(self, world, mem, threshold = 0.5, size = None):
		self.mem = mem
		if size:
			aDist = ADistWrapper(threshold, windowsize = size)
		else:
			aDist = ADistWrapper(threshold)
		aDist.init(world)
		self.mem.add(A_DIST_KEY, aDist)
		self.dimension = len(get_pred_names(world))
	
	def update(self, world):
		self.mem.update_all(A_DIST_KEY, world)
	
	def anomalous(self):
		wrappers = self.mem.get(A_DIST_KEY)
		for wrapper in wrappers:
			for pred in range(len(wrapper.trace.values[-1])):
				val, threshold = wrapper.trace.values[-1][pred]
				if val > threshold:
					return pred
		return False
	
	def clear(self):
		self.mem.remove(A_DIST_KEY)
	
	def add_WP(self, windowStart, threshold = 0.5, size = None):
		states = self.mem.get(self.mem.STATES)
		if not states:
			raise Exception("World states not in memory; cannot create new WP")
		if len(states) < windowStart + 1:
			raise Exception("Starting WP in future not allowed")
		if size:
			aDist = ADistWrapper(threshold, size, start = windowStart)
		else:
			aDist = ADistWrapper(threshold, start = windowStart)
		aDist.init(states[windowStart])
		windowStart += 1
		if windowStart + aDist.windowsize >= len(states):
			for i in range(windowStart, len(states)):
				aDist.update(states[i])
		else:
			#fill first window, then add last windows' width of data
			for i in range(windowStart, windowStart + aDist.windowsize):
				aDist.update(states[i])
			for i in range(max(i, len(states) - windowSize), len(states)):
				aDist.update(states[i])	
		self.mem.add(A_DIST_KEY, aDist)
	
	def long_str(self):
		s = ""
		n = 1
		for aDist in self.mem.get(self.memKeys.MEM_ADIST):
			s += "Window Pair " + str(n) + ". Window start time = " + str(aDist.start) + "\n" + str(aDist) + "\n"
			n += 1
		return s[:-1]

class ADistanceAnomalyNoter:
	
	'''
	MIDCA module that uses the A-distance statistical anomaly detection technique to detect anomalies in world states described in first-order [predicate] logic. This class expects a world state from the built-in MIDCA world simulator. It also expects a threshold value, which will determine the sensitivity of the anomaly detector (lower values lead to more false positives and greater recall).
	
	To extend this class to other world representations, 
	
	1) replace the get_count_vector() method at the top of the file with some method that returns a numeric vector representation of your state. Also replace all calls to that method with calls to your replacement. 
	
	2) See instructions in the ADistWrapper class for correctly initializing ADistance.
	
	3) the get_pred_names() method should return some list of the same length as the vector representing the world state. If working with GNG, ensure that for all i, get_pred_names()[i] is the name associated with the value get_count_vector()[i].
	'''
	
	def __init__(self, windowsize = None, threshold = 0.5):
		self.size = windowsize
		self.threshold = threshold
	
	def init(self, world, mem):
		self.detector = AnomalyDetector(world, mem, self.threshold, self.size)
		self.mem = mem
	
	def update(self, world):
		self.detector.update(world)
	
	def anomalous(self):
		return self.detector.anomalous()
	
	def run(self, cycle, verbose = 2):
		world = self.mem.get(self.mem.STATES)[-1]
		prevworld = copy.deepcopy(world) # for trace
		self.update(world)
		currworld = copy.deepcopy(world) # for trace
		self.mem.add(ANOMALY_STATE_KEY, self.anomalous())
		if verbose >= 1 and self.anomalous():
			print "Anomaly detected."
		elif verbose >= 2 and not self.anomalous():
			print "No anomaly detected."

		trace = self.mem.trace
		if trace:
			trace.add_module(cycle,self.__class__.__name__)
			trace.add_data("PREV WORLD", prevworld)
			trace.add_data("CURR WORLD", currworld)
			trace.add_data("ANOMALY", self.anomalous())

	#simple implementaton; will not work with multiple windows
	def __str__(self):
		s = str(self.detector)
		try:
			return s[s.rindex("\n") + 1:]
		except ValueError:
			return s

class StateDiscrepancyDetector:
	'''
	Uses Immediate Expectations to detect discrepancies.
	For now, this only looks at effects of actions.
	'''	
	def init(self, world, mem):
		self.world = world
		self.mem = mem
		
	def run(self, cycle, verbose=2):
		self.verbose = verbose
		trace = self.mem.trace
		if trace:
			trace.add_module(cycle,self.__class__.__name__)
			
		last_actions = None
		try:
			last_actions = self.mem.get(self.mem.ACTIONS)[-1]
			if self.verbose >= 2: print("last_actions are "+str(map(str,last_actions)))
			# for now assume just one action
			if len(last_actions) != 1:
				if self.verbose >= 1: print("Agent has "+str(len(last_actions))+" previous actions, will not proceed")
				if trace:
					trace.add_data("DISCREPANCY", None)
					trace.add_data("EXPECTED", None)
					trace.add_data("ACTUAL", None)
				return
		except:
			if self.verbose >= 1:
				print "No actions executed, skipping State Discrepancy Detection"
			
			return
		last_action = last_actions[0]
		copy_world = self.mem.get(self.mem.STATES)[-2]
		try:
			#print " attempting to execute action "+str(last_action)+" on preivous state:"
			#nbeacons_util.drawNBeaconsScene(copy_world)
			copy_world.apply_midca_action(last_action)
			
		except:
			print "Exception trying action "+str(last_action)+" is "+str(sys.exc_info()[0])
			print "Previous action "+str(last_action)+" not applicable, agent did not execute an action during last act phase"
		world_diff = self.world.diff(copy_world)
		
		# we don't care about activated discrepancies right now
		# beacon failures are only to ensure there is always a goal for the agent
		# remove any 'activated' beacons
		expected = world_diff[0]
		actual = world_diff[1]
		expected_no_activate = []
		i = 0
		for exp_atom in expected:
			if 'activated' in str(exp_atom):
				if self.verbose >= 1: print '  ignoring activated atoms in discrepancies'
			else:
				expected_no_activate.append(exp_atom)
			i+=1
			
		expected = expected_no_activate
	
		#print("World diff returned : "+str(world_diff))
		if len(expected) > 0 or len(actual) > 0: 
			if self.verbose >= 1: print("Expected "+str(map(str,expected))+ " but got "+str(map(str,actual)))
		else:
			if self.verbose >= 1: print "No Discrepancy Detected"
		is_discrepancy = not (len(expected) == 0 and len(actual) == 0) 
		
		if is_discrepancy:
			self.mem.set(self.mem.DISCREPANCY,(expected,actual))
		else:
			self.mem.set(self.mem.DISCREPANCY,None)
		
		if trace:
			trace.add_data("DISCREPANCY", is_discrepancy)
			trace.add_data("EXPECTED", str(map(str,expected)))
			trace.add_data("ACTUAL", str(map(str,actual)))
		
		return

class InformedDiscrepancyDetector:
	'''
	Performs Discrepancy Detection using Informed Expectations
	TODO: This is a stub for now, not implemented yet
	'''		
	def __init__(self):
		pass
	
	def init(self, world, mem):
		self.world = world
		self.mem = mem
	
	def get_current_plan(self):
		'''
		Returns the current plan the agent is using
		'''
		pass
	
	def generate_inf_exp(self, plan, prev_action):
		'''
		Returns a set of atoms to check against the state given the previous action
		the agent executed and the current plan
		
		See Dannenhauer & Munoz-Avila IJCAI-2015 / Dannenhauer, Munoz-Avila, Cox IJCAI-2016 
		for more information on informed expectations
		
		TODO: finish
		'''
		# sanity check, make sure prev_action is in the plan
		if prev_action not in plan:
			raise Exception("Cannot generate informed expectations: prev_action "+str(prev_action)+" was not given plan")
		
		exp = [] # expectations accumulator
		pass
	
	def run(self, cycle, verbose=2):
		prev_action = self.mem.get(self.mem.ACTIONS)[-1]
		curr_goals = self.mem.get(self.mem.CURRENT_GOALS)
		plan = self.mem.get(self.mem.GOAL_GRAPH).get_best_plan(curr_goals)
		inf_exp = self.generate_inf_exp(prev_action)
		for e in inf_exp:
			pass