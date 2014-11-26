import goals

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
			predicateName = txt[:txt.index("(")]
			args = [arg.strip() for arg in txt[txt.index("(") + 1:-1].split(",")]
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
			goal = self.parseGoal(val.strip())
			if goal:
				world = self.mem.get(self.mem.STATES)[-1]
				if not self.validGoal(goal, world):
					print str(goal), "is not a valid goal\nPossible predicates:", self.predicateNames(world), "\nPossible arguments", self.objectNames(world)
				else:
					self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
					print "Goal added."


	