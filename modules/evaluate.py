from MIDCA import time

class EvalPointingFromFeedback:
	
	def init(self, world, mem):
		self.mem = mem
	
	def run(self, cycle, verbose = 2):
		goals = self.mem.get(self.mem.CURRENT_GOALS)
		if not goals:
			if verbose >= 2:
				print "No current goals. Skipping eval"
		else:
			plan = self.mem.get(self.mem.GOAL_GRAPH).getMatchingPlan(goals)
			if plan and plan.finished():
				if verbose >= 1:
					print "Plan", plan, "finished. Removing it and associated goals"
				goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
				goalGraph.removePlanGoals(plan)
				goalGraph.removePlan(plan)
		

class SimpleEval:
	
	def init(self, world, mem):
		self.mem = mem
		try:
			goals = self.mem.get(self.mem.CURRENT_GOALS)
		except KeyError:
			goals = []
		if not goals:
			if verbose >= 2:
				print "No current goals. Skipping eval"
		else:
			goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
			plan = goalGraph.getMatchingPlan(goals)
			if not plan:
				if verbose >= 2:
					print "No plan found that achieves all current goals. ",
					"Skipping eval based on plan completion"
			else:
				if plan.finished():
					if verbose >= 1:
						print "Plan:", plan, "complete. Removing its goals"
					for goal in plan.goals:
						goalGraph.remove(goal)
					numPlans = len(goalGraph.plans)
					goalGraph.removeOldPlans()
					newNumPlans = len(goalGraph.plans)
					if numPlans != newNumPlans and verbose >= 1:
						print "removing", numPlans - newNumPlans, 
						"plans that no longer apply."
				else:
					if verbose >= 2:
						print "Plan:", plan, "not complete"
	
	def run(self, cycle, verbose = 2):
		world = self.mem.get(self.mem.STATES)[-1]
		try:
			goals = self.mem.get(self.mem.CURRENT_GOALS)
		except KeyError:
			goals = []
		if goals:
			for goal in goals:
				try:
					achieved = world.atom_true(world.midcaGoalAsAtom(goal))
					if 'negate' in goal and goal['negate']:
						achieved = not achieved
					if not achieved:
						if verbose >= 2:
							print "Not all goals achieved;", goal, "is not true."
						return
				except ValueError:
					if verbose >= 1:
						print "Could not test goal", goal, ". It does not seem to be a valid world state"
					return
			if verbose >= 1:
				print "All current goals achieved. Removing them from goal graph"
			goalGraph = self.mem.get(self.mem.GOAL_GRAPH)		
			for goal in goals:
				goalGraph.remove(goal)
			numPlans = len(goalGraph.plans)
			goalGraph.removeOldPlans()
			newNumPlans = len(goalGraph.plans)
			if numPlans != newNumPlans and verbose >= 1:
				print "removing", numPlans - newNumPlans, "plans that no longer apply."
		else:
			if verbose >= 2:
				print "No current goals. Skipping eval"

LAST_SCORED_GOAL = "Last Scored Goal"
SCORE = "Score"

class Score:
	
	def __init__(self):
		self.towers = 0
		self.score = 0
	
	def update(self, score):
		self.towers += 1
		self.score += score

class Scorer:
	
	'''
	MIDCA module that scores MIDCA on tower construction success. Each time a tower is built, MIDCA gets 1 point for each block in the tower (including the triangular one) that is not on fire.
	Note: This module must precede SimpleEval to work consistently.
	'''
	
	def init(self, world, mem):
		self.mem = mem
		self.world = world
		self.mem.set(LAST_SCORED_GOAL, None)
		self.mem.set(SCORE, Score())
	
	#returns one current block stacking goal, if one exists.
	def get_stacking_goal(self):
		if not self.mem.get(self.mem.CURRENT_GOALS):
			return None
		for goal in self.mem.get(self.mem.CURRENT_GOALS):
			if 'predicate' in goal and goal['predicate'] == 'on':
				return goal
		return None	
	
	def is_on_fire(self, block):
		return self.world.is_true("onfire", [block.name])
	
	def block_under(self, block):
		if self.world.is_true("on-table", [block.name]):
			return None
		for atom in self.world.atoms:
			if atom.predicate.name == "on" and atom.args[0] == block:
				return atom.args[1]
		return None
	
	def get_tower_score(self, goal):
		score = 0
		block = self.world.objects[goal.args[0]]
		while block:
			if not self.is_on_fire(block):
				score += 1
			block = self.block_under(block)
		return score
	
	def run(self, cycle, verbose = 2):
		lastGoal = self.mem.get(LAST_SCORED_GOAL)
		currentGoal = self.get_stacking_goal()
		if not currentGoal or lastGoal == currentGoal:
			return #no goal or goal already scored
		try:
			achieved = self.world.atom_true(self.world.midcaGoalAsAtom(currentGoal))
		except Exception:
			print "unable to check goal", currentGoal, ". skipping scoring"
		if 'negate' in currentGoal and currentGoal['negate']:
			achieved = not achieved
		if not achieved:
			return
		self.mem.set(LAST_SCORED_GOAL, currentGoal)
		if not self.world.is_true("triangle", [currentGoal.args[0]]):
			return #only towers with triangles on top count
		score = self.get_tower_score(currentGoal)
		self.mem.get(SCORE).update(score)
		if verbose >= 2:
			print "Tower", self.mem.get(SCORE).towers, "completed.", score, "added to score, which is now", self.mem.get(SCORE).score
		
		

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
			
			
			
