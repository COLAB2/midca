

class SimpleAct:
	
	'''
	MIDCA module that selects the plan, if any, that achieves the most current goals, then selects the next action from that plan. The selected action is stored in a two-dimensional array in mem[mem.ACTIONS], where mem[mem.ACTIONS][x][y] returns the yth action to be taken at time step x. So mem[mem.ACTIONS][-1][0] is the last action selected. Note that this will throw an index error if no action was selected.
	To have MIDCA perform multiple actions in one cycle, simple add several actions to mem[mem.ACTIONS][-1]. So mem[mem.ACTIONS][-1][0] is the first action taken, mem[mem.ACTIONS][-1][1] is the second, etc.
	'''
	
	def init(self, world, mem):
		self.mem = mem
	
	def get_next_plan(self):
		currentGoals = self.mem.get(self.mem.CURRENT_GOALS)
		#try to retrieve a plan that achieves all goals in current goal set
		plan = self.mem.get(self.mem.GOAL_GRAPH).getMatchingPlan(currentGoals)
		if plan:
			return plan
		else:
			#try to retrieve a plan that achieves at least one goal in current goal set.
			plan = self.mem.get(self.mem.GOAL_GRAPH).getBestPlan(currentGoals)
		return plan
	
	def get_valid_plan(self, world, verbose):
		plan = None
		while not plan:
			plan = self.get_next_plan()
			if not plan:
				return None #no plan retrieved; no action given
			if not world.plan_correct(plan):
				if verbose >= 1:
					print "Retrieved plan incorrect. Trying again."
				if verbose >= 2:
					print "Plan:", str(plan)
					print "Goals:", [str(goal) for goal in plan.goals]
				self.mem.get(self.mem.GOAL_GRAPH).removePlan(plan)
				plan = None
		return plan
	
	def run(self, cycle, verbose = 2):
		world = self.mem.get(self.mem.STATES)[-1]
		plan = self.get_valid_plan(world, verbose)
		if plan:
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
		else:
			if verbose >= 1:
				print "No valid plan found that achieves any current goals. MIDCA will not select an action this cycle."
			self.mem.add(self.mem.ACTIONS, [])
		
