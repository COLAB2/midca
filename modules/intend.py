
class SimpleIntend:
	
	def init(self, world, mem):
		self.mem = mem
	
	def run(self, cycle, verbose = 2):
		goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
		if not goalGraph:
			if verbose >= 1:
				print "Goal graph not initialized. Intend will do nothing."
			return
		goals = goalGraph.getUnrestrictedGoals()
		self.mem.set(self.mem.CURRENT_GOALS, goals)
		if not goals:
			if verbose >= 2:
				print "No goals selected."
		else:
			if verbose >= 2:
				print "Selecting goal(s):", 
				for goal in goals:
					print goal["objective"] + " the " + goal["directObject"],
				print