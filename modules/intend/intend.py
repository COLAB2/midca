
class SimpleIntend:
	
	def __init__(self, planOnCompletion, overrideForPriority):
		self.planOnCompletion = planOnCompletion
		self.overrideForPriority = overrideForPriority
	
	def init(self, world, mem, memKeys):
		self.mem = mem
		self.memKeys = memKeys
	
	#if new goal was generated since last run, set current goal to it. Otherwise, set current goal to None to stop redundant planning.
	def run(self, verbose = 2):
		self.mem.set(self.memKeys.MEM_CUR_GOALS, [])
		goalQueue = self.mem.get(self.memKeys.MEM_GOALS)
		nextgoals = goalQueue.next_goals()
		if self.planOnCompletion:
			#check to see if there is a current plan
			currentPlan = self.mem.get(self.memKeys.MEM_CURRENT_PLAN)
			if currentPlan and nextgoals:
				if not self.overrideForPriority or currentPlan.priority + 0.5 >= nextgoals.priority:
					#if current plan has more or less (besides time-dependent priority increases) same priority, keep current plan. Also if prioirty overrides are disabled.
					if verbose >=  2:
						print "Plan in progress. Will not send a goal to planner."
					goalQueue.add_goal_set(nextgoals)
					return
		while nextgoals and self.mem.get(self.memKeys.MEM_PLANS).goals_planned_for(nextgoals):
			if verbose >= 2:
				print "Plan exists for goal(s)", 
				for goal in nextgoals:
					print goal,
				print ". Trying next goal(s)."
			nextgoals = goalQueue.next_goals()
		self.mem.set(self.memKeys.MEM_CUR_GOALS, nextgoals)
		if nextgoals == None:
			if verbose >= 2:
				print "No goals sent to planner."
		else:
			if verbose >= 2:
				print "Sending goal(s) to planner:", 
				for goal in nextgoals:
					print goal,
				print