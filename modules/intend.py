
class SimpleIntend:
	
	def init(self, world, mem):
		self.mem = mem
	
	#if new goal was generated since last run, set current goal to it. Otherwise, set current goal to None to stop redundant planning.
	def run(self, cycle, verbose = 2):
                trace_str = "INPUT:\n"
		goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
                trace_str += "  goalgraph: "+str(self.mem.GOAL_GRAPH)
		if not goalGraph:
			if verbose >= 1:
				print "Goal graph not initialized. Intend will do nothing."
			return
		goals = goalGraph.getUnrestrictedGoals()
		self.mem.set(self.mem.CURRENT_GOALS, goals)
                trace_str += "\nOUTPUT:\n"
                trace_str += "  Current goals are now:\n "
                if goals:
                        for g in goals:
                                trace_str += "    " + str(g) + "\n"
		if not goals:
			if verbose >= 2:
				print "No goals selected."
		else:
			if verbose >= 2:
				print "Selecting goal(s):", 
				for goal in goals:
					print goal,
				print

                trace = self.mem.trace
                if trace:
                        trace.addphase(cycle,self.__class__.__name__,trace_str)
