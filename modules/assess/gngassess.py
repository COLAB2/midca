import sys
sys.path.append("../../")
from utils import gng

class AnomalyAnalyzer:

	def init(self, world, mem, memKeys, wp = 0):
		self.mem = mem
		self.numUpdates = 0	
		self.memKeys = memKeys
		self.gng = None
	
	def init_gng(self):
		self.trace = self.mem.get(self.memKeys.MEM_ADIST)[0].trace
		vector = [pair[0] for pair in self.trace.values[self.numUpdates]]
		self.gng = gng.GNG(len(vector))
		self.gng.nodeLearningRate = 0.05
		self.gng.neighborLearningRate = 0
		self.gng.maxDistance = 0.5
		self.gng.newNodeInterval = 200
		self.gng.maximumAge = 1000
	
	def updates_needed(self):
		return len(self.trace.values) - self.numUpdates
	
	#catches the analyzer up to the state of the input ADist object.
	def run(self, verbose = 2):
		if not self.gng:
			self.init_gng()
		while len(self.trace.values) > self.numUpdates:
			vector = [pair[0] for pair in self.trace.values[self.numUpdates]]
			self.gng.update(vector)
			self.numUpdates += 1
		if self.mem.get(self.memKeys.MEM_ANOM)[-1]:
			self.mem.add(self.memKeys.MEM_NODES, (self.last_node(), self.numUpdates))
			self.mem.add(self.memKeys.MEM_ANOM_TYPE, self.anomaly_type())
			if verbose >= 1:
				print "Anomaly type = " + str(self.anomaly_type())
			if verbose >= 2:
				print str(self.last_node())
		elif verbose >= 2:
			print "No anomaly: Skipping Assess."
	
	def nodes(self):
		return self.gng.nodes
	
	def last_node(self):
		vector = [pair[0] for pair in self.trace.values[-1]]
		return self.gng.closest_node(vector)[0]
	
	def anomaly_type(self):
		node = self.last_node()
		index = node.location.index(max(node.location))
		return self.trace.prednames[index]
	
	def anomaly_intensity(self, i = None):
		node = self.last_node()
		if i:
			return node.location[i] / max([n.location[i] for n in self.nodes])
		else:
			zero = [0 for i in range(len(node.location))]
			return gng.distance(node.location, zero) / max([gng.distance(n.location, zero) for n in self.nodes])
	
	def intensities(self):
		pairs = []
		for i in range(len(self.nodes[0])):
			pairs.append(self.trace.prednames[i], self.anomaly_intensity(i))
		return pairs
	
	