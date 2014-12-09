from MIDCA import goals
from _goalgen import tf_3_scen, tf_fire
from MIDCA.worldsim import blockstate

class UserGoalInput:
	
	'''
	MIDCA module that allows users to input goals in a predicate representation. These will be stored in MIDCA goals of the form Goal(arg1Name, arg2Name..., argiName, predicate = predName). Note that this class only allows for simple goals with only predicate and argument information. It does not currently check to see whether the type or number of arguments is appropriate.
	'''
	
	def init(self, world, mem):
		self.mem = mem
	
	def parseGoal(self, txt):
		if not txt.endswith(")"):
			print "Error reading goal. Goal must be given in the form: predicate(arg1, arg2,...,argi-1,argi), where each argument is the name of an object in the world"
			return None
		try:
			if txt.startswith('!'):
				negate = True
				txt = txt[1:]
			else:
				negate = False
			predicateName = txt[:txt.index("(")]
			args = [arg.strip() for arg in txt[txt.index("(") + 1:-1].split(",")]
			if negate:
				goal = goals.Goal(*args, predicate = predicateName, negate = True)
			else:
				goal = goals.Goal(*args, predicate = predicateName)
			return goal
		except Exception:
			print "Error reading goal. Goal must be given in the form: predicate(arg1, arg2,...,argi-1,argi), where each argument is the name of an object in the world"
			return None
	
	def objectNames(self, world):
		return world.objects.keys()
	
	def predicateNames(self, world):
		return world.predicates.keys()
	
	def validGoal(self, goal, world):
		try:
			for arg in goal.args:
				if arg not in self.objectNames(world):
					return False
			return goal['predicate'] in self.predicateNames(world)
		except Exception:
			return False
	
	def run(self, cycle, verbose = 2):
		if verbose == 0:
			return #if skipping, no user input
		while True:
			val = raw_input("Please input a goal if desired. Otherwise, press enter to continue\n")
			if not val:
				return "continue"
			elif val == 'q':
				return val
			goal = self.parseGoal(val.strip())
			if goal:
				world = self.mem.get(self.mem.STATES)[-1]
				if not self.validGoal(goal, world):
					print str(goal), "is not a valid goal\nPossible predicates:", self.predicateNames(world), "\nPossible arguments", self.objectNames(world)
				else:
					self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
					print "Goal added."

class TFStack:

	def init(self, world, mem):
		self.tree = tf_3_scen.Tree()
		self.mem = mem
	
	def stackingGoalsExist(self):
		graph = self.mem.get(self.mem.GOAL_GRAPH)
		for goal in graph.getAllGoals():
			if goal['predicate'] == "on":
				return True
		return False
	
	def run(self, cycle, verbose = 2):
		if self.stackingGoalsExist():
			if verbose >= 2:
				print "MIDCA already has a block stacking goal. Skipping TF-Tree stacking goal generation"
			return
		world = self.mem.get(self.mem.STATES)[-1]
		blocks = blockstate.get_block_list(world)
		goal = self.tree.givegoal(blocks)
		if goal:
			if verbose >= 2:
				print "TF-Tree goal generated:", goal
			self.mem.get(self.mem.GOAL_GRAPH).insert(goal)

class TFFire:

	def init(self, world, mem):
		self.tree = tf_fire.Tree()
		self.mem = mem
	
	def fireGoalExists(self):
		graph = self.mem.get(self.mem.GOAL_GRAPH)
		for goal in graph.getAllGoals():
			if goal['predicate'] == "onfire":
				return True
		return False
	
	def run(self, cycle, verbose = 2):
		world = self.mem.get(self.mem.STATES)[-1]
		blocks = blockstate.get_block_list(world)
		goal = self.tree.givegoal(blocks)
		if goal:
			inserted = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
			if verbose >= 2:
				print "TF-Tree goal generated:", goal,
				if inserted:
					print
				else:
					print ". This goal was already in the graph."
			
	

	