import heapq

class PlanSet:
	
	def __init__(self):
		self.plans = []
	
	#if priority is -1, orders by time added (LIFO). If -2, FIFO. Otherwise, orders by priority.
	def add_plan(self, plan):
		if plan.priority == -1:
			plan.priority = len(self.plans)
		elif plan.priority == -2:
			plan.priority = -len(self.plans)
		heapq.heappush(self.plans, plan)
	
	def get_next_plan(self):
		try:
			plan = heapq.heappop(self.plans)
			return plan
		except IndexError:
			return None
	
	def poll_next_plan(self):
		try:
			plan = heapq.heappop(self.plans)
			heapq.heappush(self.plans, plan)
			return plan
		except IndexError:
			return None	
	
	def same_goals(self, plan, goals):
		#allow check between two plans or a plan and a goal set.
		if hasattr(goals, "goals"):
			goals = goals.goals
		return goals == plan.goals
	
	def goals_planned_for(self, goals):
		for plan in self.plans:
			if self.same_goals(plan, goals):
				return True
		return False
	
	def update(self, plan):
		self.add_plan(plan)
	
	def remove_goals(self, delgoals):
		self.plans = [plan for plan in self.plans if not self.same_goals(plan, delgoals)]
		heapq.heapify(self.plans)
	
	def update_priority(self, goals, priority):
		for plan in self.plans:
			if self.same_goals(plan, goals):
				plan.priority = priority
		heapq.heapify(self.plans)