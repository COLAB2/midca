import copy

class SimpleAct:
	
	'''
	MIDCA module that selects the plan, if any, that achieves the most current goals, then selects the next action from that plan. The selected action is stored in a two-dimensional array in mem[mem.ACTIONS], where mem[mem.ACTIONS][x][y] returns the yth action to be taken at time step x. So mem[mem.ACTIONS][-1][0] is the last action selected. Note that this will throw an index error if no action was selected.
	To have MIDCA perform multiple actions in one cycle, simple add several actions to mem[mem.ACTIONS][-1]. So mem[mem.ACTIONS][-1][0] is the first action taken, mem[mem.ACTIONS][-1][1] is the second, etc.
	'''
	
	def init(self, world, mem):
		self.mem = mem
	
	#returns the plan that achieves the most current goals, based on simulation.
	def get_best_plan(self, world, goals, verbose):
		plan = None
		goalsAchieved = set()
		goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
		for nextPlan in goalGraph.allMatchingPlans(goals):
			achieved = world.goals_achieved(nextPlan, goals)
			if len(achieved) > len(goalsAchieved):
				goalsAchieved = achieved
				plan = nextPlan
			if len(achieved) == len(goals):
				break
			elif verbose >= 1:
				print "Retrieved plan does not achieve all goals. Trying again."
				if verbose >= 2:
					print "Plan:", str(nextPlan)
					print "Goals achieved:", [str(goal) for goal in achieved]
		if plan == None and verbose >= 1:
			print "No valid plan found that achieves any current goals."
		elif len(goalsAchieved) < len(goals) and verbose >= 1:
			print "Best plan does not achieve all goals."
			if verbose >= 2:
				print "Plan:", str(plan)
				print "Goals achieved:", [str(goal) for goal in goalsAchieved]
		return plan
	
	def run(self, cycle, verbose = 2):
		world = self.mem.get(self.mem.STATES)[-1]
		goals = self.mem.get(self.mem.CURRENT_GOALS)
		plan = self.get_best_plan(world, goals, verbose)

                trace = self.mem.trace
                if trace:
                        trace.add_module(cycle,self.__class__.__name__)
                        trace.add_data("WORLD", copy.deepcopy(world))
                        trace.add_data("GOALS", copy.deepcopy(goals))
                        trace.add_data("PLAN", copy.deepcopy(plan))                        

		if plan != None:
			action = plan.get_next_step()
			if not action:
				if verbose >= 1:
					print "Plan to achieve goals has already been completed. Taking no action."
				self.mem.add(self.mem.ACTIONS, [])
			else:
				if verbose == 1:
					print "Action selected:", action
				elif verbose >= 2:
					print "Selected action", action, "from plan:\n", plan
				self.mem.add(self.mem.ACTIONS, [action])
				plan.advance()

                                if trace: trace.add_data("ACTION", action)
		else:
			if verbose >= 1:
				print "MIDCA will not select an action this cycle."
			self.mem.add(self.mem.ACTIONS, [])

                        if trace: trace.add_data("ACTION", None)

		
