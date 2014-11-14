
class Goal:
	
	def __init__(self, *args, **kwargs):
		if 'id' in kwargs:
			self.id = kwargs['id']
		else:
			self.id = hash(self)
		self.args = args
		self.kwargs = kwargs
	
	def __getitem__(self, val):
		if val in self.kwargs:
			return self.kwargs[val]
		else:
			try:
				return self.args[val]
			except TypeError:
				#not an index
				raise TypeError(str(val) + " is not a valid key or index.")

class GoalNode:
	
	def __init__(self, goal):
		self.goal = goal
		self.parents = set()
		self.children = set()
		self.plan = None
	
	def addChild(self, node):
		self.children.add(node)
		node.parents.add(self)
	
	def setPlan(self, plan):
		self.plan = plan

class GoalGraph:
	
	def __init__(self, goalCompareFunction):
		self.roots = []
		self.cmp = goalCompareFunction
		self.numGoals = 0
	
	#note not symmetrical - finds goals that are specifications of current goal, but not generalizations.
	def consistentGoal(self, first, second):
		for i in len(first.args):
			if first.args[i] != "?" and first.args[i] != second.args[i]:
				return False
		for key, val in first.kwargs.items():
			if key not in second.kwargs or second.kwargs[key] != val:
				return False
		return True
	
	def insert(self, goal):
		newNode = GoalNode(goal)
		self.numGoals += 1
		if not self.roots:
			self.roots.append(newNode)
		for node in self.getAllNodes():
			cmpVal = self.cmp(newNode, node)
			if cmpVal < 0:
				newNode.addChild(node)
			elif cmpVal > 0:
				node.addChild(newNode)
		self.roots = [node for node in self.roots if node not in newNode.children]
		if not newNode.parents:
			self.roots.add(newNode)
		if not self.roots
	
	def _removeNode(self, delNode):
		if delNode in self.roots:
			self.roots.remove(delNode)
			for node in self.getAllNodes():
				if delNode = node.parents:
					node.parents.remove(delNode)
					if not node.parents:
						self.roots.add(node)
				if delNode in node.children:
					node.children.remove(delNode)
	
	def remove(self, goal):
		delNode = self._getGoalNode(goal)
		if not delNode:
			return
		self._removeNode(delNode)
		self.remove(goal) #in case goal added more than once
	
	def addPlan(self, goals, plan):
		for goal in goals:
			node = self._getGoalNode(goal)
			if not node:
				raise ValueError(str(goal) + " not in goal graph."
			node.setPlan(plan)
	
	#removes all goals associated with given plan
	def removePlanGoals(self, plan):
		for node in self.getAllNodes():
			if node.plan == plan:
				self._removeNode(node)
	
	def getAllNodes(self):
		visited = set()
		nodes = list(self.roots)
		while nodes:
			next = nodes.pop(0)
			if next in visited:
				continue
			else:
				visited.add(next)
			for child in next.children:
				nodes.append(child)
		return visited
	
	def getAllGoals(self):
		visited = set()
		goals = []
		nodes = list(self.roots)
		while nodes:
			next = nodes.pop(0)
			if next in visited:
				continue
			else:
				visited.add(next)
			goals.append(next.goal)
			for child in next.children:
				nodes.append(child)
		return goals
	
	#returns the first node such that self.consistentGoal(goal, node.goal) returns True.
	def _getGoalNode(self, goal):
		visited = set()
		nodes = list(self.roots)
		while nodes:
			next = nodes.pop(0)
			if next in visited:
				continue
			else:
				visited.add(next)
			if self.consistentGoal(goal, next.goal):
				return next
			for child in next.children:
				nodes.append(child)
		return None #not in graph
	
	def getGoalAncestors(self, goal):
		node = self._getGoalNode(goal)
		if node:
			ancestors = set()
			nodes = [node]
			while nodes:
				next = nodes.pop(0)
				if next in ancestors:
					continue
				for parent in next.parents:
					ancestors.add(parent)
					nodes.append(parent)
			return ancestors
		else:
			raise ValueError("Goal not in graph")
		
	def __contains__(self, goal):
		return val in self.getAllGoals()
	
	def __str__(self):
		return "Goals: " + str([str(goal) + " " for goal in self.getAllGoals()])
	
	def getUnrestrictedGoals(self):
		return list(self.roots)
	
	#returns the plan associated with the most root [unrestricted] nodes. If there is no plan associated with any of them, will return None. Ties are broken arbitrarily.
	def getBestPlan(self):
		choices = {}
		for node in self.roots:
			if node.plan:
				if node.plan in choices:
					choices[node.plan] += 1
				else:
					choices[node.plan] = 1
		if not choices:
			return None
		maxVal = max(choices.values())
		for choice, count in choices.items():
			if count == maxVal:
				return choice