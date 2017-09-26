import math

def euclidian_distance(v1, v2):
	assert len(v1) == len(v2)
	total = 0.0
	for i in range(len(v1)):
		total += (v1[i] - v2[i]) ** 2
	return math.sqrt(total)

def distance(v1, v2):
	return euclidian_distance(v1, v2)

class Node:
	
	def __init__(self, dimensionality, location = None):
		if location:
			self.location = []
			for val in location:
				self.location.append(val)
		else:
			self.location = [0] * dimensionality
		self.error = 0.0
		self.updates = 0
		self.importance = 6
	
	def move_towards(self, v, learnrate):
		assert len(v) == len(self.location)
		for i in range(len(v)):
			self.location[i] += learnrate * (v[i] - self.location[i])
		if learnrate > 0:
			self.updates += 1
			self.importance = self.importance * 1.01 + 6
	
	def __str__(self):
		dim = min(8, len(self.location))
		s = "["
		for i in range(dim):
			s += str(round(self.location[i], 2)) + " "
		s = s[:-1] + "]"
		return s

class Edge:
	
	def __init__(self, node1, node2):
		self.node1 = node1
		self.node2 = node2
		self.age = 0
	
	def __eq__(self, other):
		return self.node1 in [other.node1, other.node2] and self.node2 in [other.node1, other.node2]
	
	def __str__(self):
		s = "Edge: "
		s += str(self.node1) + " <----> " + str(self.node2)
		return s

class GNG:
	
	def __init__(self, dimensionality, loc1 = None, loc2 = None, addmode = "new", delmode = "old"):
		self.nodes = []
		self.edges = []
		self.addmode = addmode
		self.newNodeInterval = 100;
		self.nodeLearningRate = 0.2;
  		self.neighborLearningRate = 0.06;
		self.errorDecay = 0.995;
		self.newNodeErrorDecay = 0.5;
		self.maximumAge = 50;
		self.numUpdates = 0;
		self.maximumNodes = 100;
		self.maxDistance = 0.1
		self.minDistance = 0.05
		self.delMode = delmode
		self.delFrequency = 2
		if loc1:
			assert len(loc1) == dimensionality
			self.nodes.append(Node(dimensionality, loc1))
		else:
			self.nodes.append(Node(dimensionality))
		if loc2:
			assert len(loc2) == dimensionality
			self.nodes.append(Node(dimensionality, loc2))
		else:
			self.nodes.append(Node(dimensionality))
		self.edges.append(Edge(self.nodes[1], self.nodes[0]))
	
	def list_params(self):
		pass #implement eventually - list params and values
	
	def closest_node(self, v, exclude = []):
		mindist = float("inf")
		closest = None
		for node in self.nodes:
			if node in exclude:
				continue
			dist = distance(node.location, v)
			if dist < mindist:
				mindist = dist
				closest = node
		return closest, mindist

	def max_error_node(self, nodeset = None):
		if not nodeset:
			nodeset = self.nodes
		maxErrorNode = None
		maxError = float("-inf")
		for node in nodeset:
			if node.error > maxError:
				maxErrorNode = node
				maxError = node.error
		return maxErrorNode
	
	def get_neighbors(self, node):
		neighbors = []
		for edge in self.edges:
			if edge.node1 == node:
				neighbors.append(edge.node2)
			elif edge.node2 == node:
				neighbors.append(edge.node1)
		return neighbors
	
	def incident_edges(self, node):
		edges = []
		for edge in self.edges:
			if edge.node1 == node or edge.node2 == node:
				edges.append(edge)
		return edges
	
	def add_node(self):
		node = self.max_error_node()
		neighbor = self.max_error_node(self.get_neighbors(node))
		newnode = Node(len(node.location), node.location)
		self.nodes.append(newnode)
		newnode.move_towards(neighbor.location, 0.5) #split the difference
		self.edges.append(Edge(newnode, node))
		self.edges.append(Edge(newnode, neighbor))
		self.edges.remove(Edge(node, neighbor))
		node.error *= self.newNodeErrorDecay
		neighbor.error *= self.newNodeErrorDecay
		newnode.error = node.error
	
	def update(self, v):
		self.numUpdates += 1
		if len(self.nodes) < 2:
			self.nodes.append(Node(len(v), v))
			self.edges.append(Edge(self.nodes[0], self.nodes[1]))
			return
		closest, closestdist = self.closest_node(v)
		nextClosest, nextclosestdist = self.closest_node(v, [closest])
		
		#update edge ages
		if self.delMode != "new":
			for edge in self.incident_edges(closest):
				edge.age += 1
		#update node ages instead
		else: #potentially delete 1 node if two are close
			for node in self.nodes:
				break
				closestother, otherdist = self.closest_node(node.location, [node])
				if closestother:
					if distance(node.location, closestother.location) < self.minDistance:
						if node.importance > closestother.importance:
							if closestother.updates > 50:
								self.nodes.remove(closestother)
						else:
							if node.updates > 50:
								self.nodes.remove(node)
						break
		#update closest error
		closest.error += distance(closest.location, v) ** 2
		#move nodes towards input
		closest.move_towards(v, self.nodeLearningRate)
		#for neighbor in self.get_neighbors(closest):
			#neighbor.move_towards(v, self.neighborLearningRate)
		#reset edge age between two closest, if exists. Otherwise create it.
		if Edge(closest, nextClosest) in self.edges:
			self.edges.remove(Edge(closest, nextClosest))
		self.edges.append(Edge(closest, nextClosest))
		
		#remove old edges, nodes with no edges - old method
		if self.delMode != "new":
			self.edges = [edge for edge in self.edges if edge.age <= self.maximumAge]
			nodes = {}
			for edge in self.edges:
				nodes[edge.node1] = True
				nodes[edge.node2] = True
			if len(self.nodes) != len(nodes):
				pass#print "Removing node"
			self.nodes = nodes.keys()
		else: #new del method
			newnodes = [node for node in self.nodes if node.importance > 0]
			#for node in self.nodes:
				#if node not in newnodes:
					#print "deleting", node
			#self.nodes = newnodes
							
		#add node - old method
		if self.addmode != "new":
			if len(self.nodes) < self.maximumNodes and self.numUpdates % self.newNodeInterval == 0:
				print "adding"
				self.add_node()
		else:
			#add node - new method
			if distance(closest.location, v) > self.maxDistance and len(self.nodes) < self.maximumNodes:
				self.nodes.append(Node(len(v), v))
				self.edges.append(Edge(self.nodes[-1], closest))
		for node in self.nodes:
			node.error *= self.errorDecay
		