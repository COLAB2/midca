from MIDCA import goals, time
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
			#use on-table predicate
			if predicateName == 'on' and len(args) == 2 and 'table' == args[1]:
				predicateName = 'on-table'
				args = args[:1]
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

	'''
	MIDCA module that generates goals to stack blocks using Michael Maynord's TF-Trees. These trees are trained to cycle through 3 specific states; behavior is unknown for other states. See implementation in modules/_goalgen/tf_3_scen.py for details.
	'''

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

	'''
	MIDCA module that generates goals to put out fires using Michael Maynord's TF-Trees. The behavior is as follows: if any fires exist, a single goal will be generated to put out a fire on some block that is currently burning. Otherwise no goal will be generated.
	'''

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

class ReactiveApprehend:
	
	'''
	MIDCA module that generates a goal to apprehend an arsonist if there is one who is free and there is a fire in the current world state. This is designed to simulate the behavior of the Meta-AQUA system.
	'''
	
	def init(self, world, mem):
		self.mem = mem
	
	def free_arsonist(self):
		world = self.mem.get(self.mem.STATES)[-1]
		for atom in world.atoms:
			if atom.predicate.name == "free" and atom.args[0].type.name == "ARSONIST":
				return atom.args[0].name
		return False
	
	def is_fire(self):
		world = self.mem.get(self.mem.STATES)[-1]
		for atom in world.atoms:
			if atom.predicate.name == "onfire":
				return True
		return False
	
	def run(self, cycle, verbose = 2):
		arsonist = self.free_arsonist()
		if arsonist and self.is_fire():
			goal = goals.Goal(arsonist, predicate = "free", negate = True)
			inserted = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
			if verbose >= 2:
				print "Meta-AQUA simulation goal generated:", goal,
				if inserted:
					print
				else:
					print ". This goal was already in the graph."

class InstructionReceiver:
	
	def init(self, world, mem):
		self.mem = mem
		self.lastTime = time.now()
	
	def run(self, cycle, verbose = 2):
		world = self.mem.get(self.mem.STATE)
		i = len(world.utterances)
		while i > 0:
			if self.lastTime - world.utterances[i - 1].time > 0:
				break
			i -= 1
		newUtterances = [utterance.utterance for utterance in world.utterances[i:]]
		#now add goals based on new utterances
		for utterance in newUtterances:
			if verbose >= 2:
				print "received utterance:", utterance
			if utterance == "point to the quad" or utterance == "goodbye baxter":
				goal = goals.Goal(objective = "show-loc", subject = "self", 
				directObject = "quad", indirectObject = "observer")
				added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
				if verbose >= 2:
					if added:
						print "adding goal:", str(goal)
					else:
						print "generated goal:", str(goal), "but it is already in the \
						goal graph"
			if utterance == "get the red block":
				goal = goals.Goal(objective = "holding", subject = "self", 
				directObject = "red block", indirectObject = "observer", pos = 'red block:table')
				added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
				if verbose >= 2:
					if added:
						print "adding goal:", str(goal)
					else:
						print "generated goal:", str(goal), "but it is already in the \
						goal graph"
			if utterance == "get the blue block":
				goal = goals.Goal(objective = "holding", subject = "self", 
				directObject = "blue block", indirectObject = "observer")
				added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
				if verbose >= 2:
					if added:
						print "adding goal:", str(goal)
					else:
						print "generated goal:", str(goal), "but it is already in the \
						goal graph"
			
			if utterance == "get the green block":
				goal = goals.Goal(objective = "holding", subject = "self", 
				directObject = "green block", indirectObject = "observer", pos = 'green block:table')
				added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
				if verbose >= 2:
					if added:
						print "adding goal:", str(goal)
					else:
						print "generated goal:", str(goal), "but it is already in the \
						goal graph"
			
			if utterance == "stack green block on red block":
				goal = goals.Goal(objective = "holding", subject = "self", 
				directObject = "green block", indirectObject = "observer", pos = 'green block:red block')
				added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
				if verbose >= 2:
					if added:
						print "adding goal:", str(goal)
					else:
						print "generated goal:", str(goal), "but it is already in the \
						goal graph"
						
			if utterance == "stack red block on green block":
				goal = goals.Goal(objective = "holding", subject = "self", 
				directObject = "green block", indirectObject = "observer", pos = 'red block:green block')
				added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
				if verbose >= 2:
					if added:
						print "adding goal:", str(goal)
					else:
						print "generated goal:", str(goal), "but it is already in the \
						goal graph"
						
# 			else:
# 				print "message is unknown"
							
		self.lastTime = time.now()
		