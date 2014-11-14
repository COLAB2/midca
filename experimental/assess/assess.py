from valence import *
from gngassess import *
#imports include values of memory keys used below

class Assessor:
	
	def __init__(self, windowsize, threshold, valenceAssess):
		self.size = windowsize
		self.threshold = threshold
		self.valenceActive = valenceAssess
	
	def init(self, world, mem):
		if self.valenceActive:
			self.valenceAssess = PredicateChangeAssessor(self.size)
			self.valenceAssess.init(world, mem)
		else:
			self.valenceAssess = None
		self.gngAssess = AnomalyAnalyzer()
		self.gngAssess.init(world, mem)
		self.mem = mem
		self.MAAnomCount = 1
	
	def lisp_anom_str(self):
		self.predi = self.mem.get(ANOMALY_MEM_KEY)[-1]
		s = "(anomaly." + str(self.MAAnomCount) + "\n"
		anompred = self.mem.get(ANOMALY_TYPE_KEY)[-1]
		s += "\t(predicate (value " + anompred + "))\n"
		anomvalence = self.mem.get(VALENCE_KEY)
		s += "\t(valence-class (value " + anomvalence + "))\n"
		anomintensity = max(self.mem.get(GNG_NODES)[-1][0].location)
		if anomintensity < self.threshold * 1.3:
			s += "\t(intensity (value low))\n"
		elif anomintensity < self.threshold * 1.5:
			s += "\t(intensity (value medium))\n"
		else:
			s += "\t(intensity (value high))\n"
		anomrange = len(self.mem.get(self.mem.STATES)) - self.size, len(self.mem.get(self.mem.STATES))
		s += "\t(window-range (start " + str(anomrange[0]) + ") (end " + str(anomrange[1]) + ")))"
		return s
	
	def run(self, cycle, verbose = 2):
		if self.valenceAssess:
			self.valenceAssess.run(cycle, verbose)
			if self.mem.get(ANOMALY_STATE_KEY) and self.mem.get(ANOMALY_STATE_KEY)[-1]:
				print "M-A frame:"
				print self.lisp_anom_str()
				self.MAAnomCount += 1
		self.gngAssess.run(cycle, verbose)
		
			