
NORMAL_BLOCK_VAL = 1.0
ON_FIRE_BLOCK_VAL = 0.0

class Evaluator:
	
	def __init__(self, restartFires = True):
		self.restartFires = restartFires
	
	def init(self, world, mem, memKeys):
		self.mem = mem
		self.actualWorld = world
		self.towersFinished = 0
		self.fireturns = 0
		self.score = 0
		self.memKeys = memKeys
	
	def num_fires(self, world):
		n = 0
		for atom in world.atoms:
			if atom.predicate.name == "onfire":
				n += 1
		return n
	
	def num_fires_small(self, world, blocks = ("A_", "B_", "D_")):
		return len([1 for block in blocks if world.is_true("onfire", [block])])
	
	def put_out_fires(self, verbose = 2):
		if verbose >= 2:
			print "putting out fires"
		self.actualWorld.atoms = [atom for atom in self.actualWorld.atoms if atom.predicate.name != "onfire"]
		world = self.mem.get(self.memKeys.MEM_STATES)[-1]
		world.atoms = [atom for atom in world.atoms if atom.predicate.name != "onfire"]
	
	def run(self, verbose = 2):
		world = self.mem.get(self.memKeys.MEM_STATES)[-1]
		self.fireturns += self.num_fires(world)
		currentPlan = self.mem.get(self.memKeys.MEM_CURRENT_PLAN)
		if currentPlan:
			currentgoals = currentPlan.goals
		else:
			currentgoals = None
		accomplished = True
		if currentgoals:
			if not hasattr(currentgoals, "__iter__"):
				currentgoals = [currentgoals]
			for currentgoal in currentgoals:
				accomplishedthis = False
				if currentgoal.goaltype == "on":
					accomplishedthis = world.is_true("on", [arg.id for arg in currentgoal.goalargs])
					if currentgoal.goalargs[0].id == "D_" and currentgoal.goalargs[1].id == "C_" and accomplishedthis:
						pass
					elif currentgoal.goalargs[0].id == "D_" and currentgoal.goalargs[1].id == "B_" and accomplishedthis:
						pass
				elif currentgoal.goaltype == "apprehend":
					accomplishedthis = not world.is_true("free", [arg for arg in currentgoal.goalargs])
				elif currentgoal.goaltype == "notonfire":
					accomplishedthis = not world.is_true("onfire", [arg.id for arg in currentgoal.goalargs])
				elif currentgoal.goaltype == "arm-empty":
					accomplishedthis = world.is_true("arm-empty", [arg.id for arg in currentgoal.goalargs])
				accomplished = accomplished and accomplishedthis
			if verbose >= 1:
				s = "current goals: " + "".join([str(currentgoal) + " " for currentgoal in currentgoals]) + " have "
				if not accomplished:
					s += "not "
				s += "been accomplished."
				print s
		else:
			if verbose >= 2:
				print "No current goal. Skipping Eval."
		if world.is_true("on", ["D_", "C_"]):
			if verbose >= 2:
				print "Tall Tower completed"
			self.towersFinished += 1
			self.score += (4 - self.num_fires(world)) * NORMAL_BLOCK_VAL + self.num_fires(world) * ON_FIRE_BLOCK_VAL
			if self.restartFires:
				self.put_out_fires(verbose)
		elif world.is_true("on", ["D_", "B_"]):
			if verbose >= 2:
				print "Short Tower completed"
			self.towersFinished += 1
			self.score += (3 - self.num_fires_small(world)) * NORMAL_BLOCK_VAL + self.num_fires_small(world) * ON_FIRE_BLOCK_VAL
			if self.restartFires:
				self.put_out_fires(verbose)
		if verbose >= 2:
			print "Total towers built:", self.towersFinished
			print "Total fire turns:", self.fireturns
			print "Total score:", self.score
		self.mem._update(self.memKeys.MEM_GOAL_COMPLETED, accomplished)
		if accomplished and currentPlan:
			if verbose >= 2:
				print "Goal completed for current plan. Removing all intended goals and pending plans for this goal."
			self.mem._update(self.memKeys.MEM_OLD_PLANS, currentPlan)
			self.mem.set(self.memKeys.MEM_CURRENT_PLAN, None)
			self.mem.get(self.memKeys.MEM_PLANS).remove_goals(currentgoals)
			self.mem.get(self.memKeys.MEM_GOALS).remove_goal_set(currentgoals)
			
			
			