import copy
import adist
from adist import ADistance, ChangeFinder, WindowPair, Interval

def get_pred_names(world):
	return sorted(world.predicates.keys())

def get_pred_counts(world):
	counts = {}
	for name in get_pred_names(world):
		counts[name] = 0
	for atom in world.atoms:
		counts[atom.predicate.name] += 1
	return counts

def get_count_vector(world):
	counts = get_pred_counts(world)
	vector = []
	for name in get_pred_names(world):
		vector.append(counts[name])
	return vector

class ADistTrace:
	
	def __init__(self, prednames, startTime = 1):
		self.prednames = prednames
		self.values = [] #tuples: value, theshold
		self.startTime = startTime
	
	def update(self, distobjs):
		stepvals = []
		for cf in distobjs:
			stepvals.append((cf.getDistances()[0], cf.windowPairs[0].alpha))
		self.values.append(stepvals)
	
	def sequences(self):
		sequences = []
		for name in self.prednames:
			sequences.append([])
		for step in self.values:
			for i in range(len(step)):
				sequences[i].append(step[i][0])
		return sequences
	
	def val_to_str(self, val, threshold):
		if val > threshold:
			return '\033[94m' + str(round(val, 2)) + '\033[0m'
		else:
			return str(round(val, 2))
	
	def short_str(self, length, threshold = None):
		s = "time"
		for name in [self.prednames[0]] + self.prednames[3:5]:
			s += "\t|\t" + name[:7]
		i = self.startTime
		if len(self.values) > length:
			startnum = length / 2
			endnum = length - startnum
			values = self.values[:startnum] + self.values[-endnum:]
		else:
			values = self.values
			startnum = len(values) + 1
			endnum = 0
		for line in values:
			s += "\n" + str(i)
			for val in [line[0]] + line[3:5]:
				if threshold:
					valstr = self.val_to_str(val[0], threshold)
				else:
					valstr = self.val_to_str(val[0], val[1])
				s += "\t|\t" + valstr
			if i == startnum:
				print "\n...\n..."
				i = len(self.values) - endnum
			i += 1
		return s
	
	def __str__(self):
		return self.short_str(100)

class ADistWrapper:
	
	#This class is designed for simple domain change representations where values for predicate count changes are resticted to [-1, 0, 1].
	#Also, currently only the simple two-window approach is supported.
	def __init__(self, threshold, windowsize = 10, start = 1):
		self.distobjs = []	#a-distance	objects, ordered by predicate
		self.threshold = threshold
		self.windowsize = windowsize
		self.start = start
	
	def init_adist(self, prednames):
		for name in prednames:
			distobj = ADistance.ADistance()
			cf = ChangeFinder.ChangeFinder(distobj)
			for i in [-1.5, -0.5, 0.5]:
				distobj.add(Interval.Interval(i, i + 1))
			cf.addWindowPair(WindowPair.WindowPair(self.windowsize, self.windowsize, self.threshold))
			distobj.init(cf)
			self.distobjs.append(cf)
	
	def init(self, world):
		self.prednames = get_pred_names(world)
		self.init_adist(self.prednames)
		self.lastPredCounts = get_count_vector(world)
		self.trace = ADistTrace(self.prednames, self.start)
	
	def update(self, world):
		newPredCounts = get_count_vector(world)
		countdifs = [new - old for new, old in zip(newPredCounts, self.lastPredCounts)]
		for i in range(len(countdifs)):
			self.distobjs[i].addData(countdifs[i])
		self.trace.update(self.distobjs)
		self.lastPredCounts = newPredCounts
	
	def __str__(self):
		return str(self.trace)
	