from MIDCA.modules._plan.asynch import asynch

class AsynchronousAct:
	
	'''
	MIDCA module that "executes" plans in which the individual actions will be conducted
	asynchronously. This was originally designed to allow MIDCA to work as a robot
	controller in communication with ROS sensor and effector nodes.
	'''
	
	def init(self, world, mem):
		self.mem = mem
	
	def run(self, cycle, verbose = 2):		
		goals = self.mem.get(self.mem.CURRENT_GOALS)
		if not goals:
			if verbose >= 2:
				print "No Active goals. Act phase will do nothing"
			return
		
		try:
			plan = self.mem.get(self.mem.GOAL_GRAPH).getMatchingPlan(goals)
		except:
			if verbose >= 1:
				print "Error loading plan. Skipping act phase."
			return

		if not plan:
			if verbose > 2:
				print "No current plan. Skipping Act phase"
			return
		i = 0
		if plan.finished():
			print "Plan", plan, "has already been completed"
			return
		#ideally MIDCA should check for other valid plans, but for now it doesn't.	
		
		while i < len(plan):
			action = plan[i]
			try:
				if action.status != asynch.FAILED and action.status != asynch.COMPLETE:
					completed = action.check_complete()
					if completed:
						if verbose >= 2:
							print "Action", action, "completed"
			except AttributeError:
				if verbose >= 1:
					print "Action", action, "Does not seem to have a valid check_complete() ",
					"method. Therefore MIDCA cannot execute it."
					action.status = asynch.FAILED
			try:
				if action.status == asynch.NOT_STARTED:
					if verbose >= 2:
						print "Beginning action execution for", action
					action.execute()
			except AttributeError:
				if verbose >= 1:
					print "Action", action, "Does not seem to have a valid execute() ",
					"method. Therefore MIDCA cannot execute it"
					action.status = asynch.FAILED
			if action.status == asynch.COMPLETE:
				i += 1
			elif not action.blocks:
				i += 1
			else:
				break
		
		

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
		else:
			if verbose >= 1:
				print "MIDCA will not select an action this cycle."
			self.mem.add(self.mem.ACTIONS, [])
		
