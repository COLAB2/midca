import heapq



class GoalQueue:
	
	def __init__(self):
		self.goals = set()
		self.goalQueue = []
	
	def add_goal_set(self, goals, priority = None):
		goalSet = GoalSet(goals, priority)
		if goalSet in self.goals:
			#print "adjusting"
			self.adjust_priority(goalSet)
		else:
			heapq.heappush(self.goalQueue, goalSet)
			self.goals.add(goalSet)
		#print "goals:", [str(s) for s in self.goals]
		#print "queue:", [str(s) for s in self.goalQueue]
	
	#adds goal if it is not in goal queue. Otherwise, adjusts to new priority.
	def add_goal(self, goal):
		self.add_goal_set([goal])
	
	def next_goals(self):
		try:
			goalSet = heapq.heappop(self.goalQueue)
			self.goals.remove(goalSet)
			return goalSet
		except IndexError:
			return []
	
	def remove_goal_set(self, goals):
		goalSet = GoalSet(goals)
		if goalSet in self.goals:
			self.goalQueue.remove(goalSet)
			self.goals.remove(goalSet)
	
	def remove_goal(self, goal):
		self.remove_goal_set([goal])
	
	def adjust_priority(self, goalSet):
		for goal2 in self.goals:
			if goalSet == goal2:
				goal2.priority = goalSet.priority
		heapq.heapify(self.goalQueue)
	
	def update(self, goals):
		self.add_goal_set(goals)