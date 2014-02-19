

class Exec:
	
	def init(self, world, mem, memKeys):
		self.mem = mem
		self.memKeys = memKeys
	
	def plan_valid(self, plan, world):
		testWorld = world.copy()
		for action in plan.get_remaining_steps():
			if not testWorld.action_applicable(action):
				print action
				print world
				return False
			testWorld.apply_action(action.op, action.args)
		return True
	
	def get_next_plan(self):
		return self.mem.get(self.memKeys.MEM_PLANS).get_next_plan()
	
	def get_action(self, world, verbose):
		plan = self.get_next_plan()
		currentPlan = self.mem.get(self.memKeys.MEM_CURRENT_PLAN)
		if not currentPlan:
			#if no old plan exists, use the new plan.
			currentPlan = plan
		elif plan and (plan.priority > currentPlan.priority):
			#if the new plan is higher priority, use the new plan and save the old.
			self.mem._update(self.memKeys.MEM_PLANS, currentPlan)
			currentPlan = plan
		elif plan:
			#if old is >= new priority, save new, keep old.
			self.mem._update(self.memKeys.MEM_PLANS, plan)
		if not currentPlan:
			return None
		while currentPlan.finished() or not self.plan_valid(currentPlan, world):
			#save finished/failed plan in mem. -1 priority signals to store by order added.
			currentPlan.priority = -1
			self.mem._update(self.memKeys.MEM_OLD_PLANS, currentPlan)
			if verbose >= 2:
				if currentPlan.finished():
					print "loaded already completed plan. Trying next in queue."
				else:
					print "loaded invalid plan. Trying next in queue."
			currentPlan = self.get_next_plan()
			if not currentPlan:
				return None
		action = currentPlan.get_next_step()
		return action, currentPlan
	
	def run(self, verbose = 2):
		world = self.mem.get(self.memKeys.MEM_STATES)[-1]
		res = self.get_action(world, verbose)
		try:
			action, plan = res
			self.mem._add(self.memKeys.MEM_ACTIONS, action)
			self.mem.set(self.memKeys.MEM_CURRENT_PLAN, plan)
			if verbose == 1:
				print "Executing action " + str(action)
			elif verbose >= 2:
				print "Executing action " + str(action) + " from plan:\n" + plan.last_step_str()
		except TypeError: #no action found
			if verbose >= 1:
				print "No action selected; all plans are complete or invalid."
			self.mem._add(self.memKeys.MEM_ACTIONS, None)
			self.mem.set(self.memKeys.MEM_CURRENT_PLAN, None)
		
