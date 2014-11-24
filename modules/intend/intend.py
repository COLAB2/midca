
class SimpleIntend:
	
	def init(self, world, mem):
		self.mem = mem
	
	#if new goal was generated since last run, set current goal to it. Otherwise, set current goal to None to stop redundant planning.
	def run(self, verbose = 2):
		goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
		if not goalGraph:
			if verbose >= 1:
				print "Goal graph not initialized. Intend will do nothing."
			return
		goals = goalQueue.getUnrestrictedGoals()
		self.mem.set(self.mem.CURRENT_GOALS, goals)
		if not goals:
			if verbose >= 2:
				print "No goals sent to planner."
		else:
			if verbose >= 2:
				print "Sending goal(s) to planner:", 
				for goal in goals:
					print goal,
				print