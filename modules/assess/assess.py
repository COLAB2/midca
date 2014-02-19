import sys
sys.path.append("../")
import gngassess, valence

class Assessor:
	
	def __init__(self, windowsize, threshold, valenceAssess):
		self.size = windowsize
		self.threshold = threshold
		self.valenceActive = valenceAssess
	
	def init(self, world, mem, memKeys):
		if self.valenceActive:
			self.valenceAssess = valence.PredicateChangeAssessor(self.size)
			self.valenceAssess.init(world, mem)
		else:
			self.valenceAssess = None
		self.gngAssess = gngassess.AnomalyAnalyzer()
		self.gngAssess.init(world, mem, memKeys)
		self.mem = mem
		self.memKeys = memKeys
		self.MAAnomCount = 1
	
	def lisp_anom_str(self):
		self.predi = self.mem.get(self.memKeys.MEM_ANOM)[-1]
		s = "(anomaly." + str(self.MAAnomCount) + "\n"
		anompred = self.mem.get(self.memKeys.MEM_ANOM_TYPE)[-1]
		s += "\t(predicate (value " + anompred + "))\n"
		anomvalence = self.mem.get(self.memKeys.MEM_VALENCE)
		s += "\t(valence-class (value " + anomvalence + "))\n"
		anomintensity = max(self.mem.get(self.memKeys.MEM_NODES)[-1][0].location)
		if anomintensity < self.threshold * 1.3:
			s += "\t(intensity (value low))\n"
		elif anomintensity < self.threshold * 1.5:
			s += "\t(intensity (value medium))\n"
		else:
			s += "\t(intensity (value high))\n"
		anomrange = len(self.mem.get(self.memKeys.MEM_STATES)) - self.size, len(self.mem.get(self.memKeys.MEM_STATES))
		s += "\t(window-range (start " + str(anomrange[0]) + ") (end " + str(anomrange[1]) + ")))"
		return s
	
	def run(self, verbose = 2):
		if self.valenceAssess:
			self.valenceAssess.run(verbose)
			if self.mem.get(self.memKeys.MEM_ANOM) and self.mem.get(self.memKeys.MEM_ANOM)[-1]:
				print "M-A frame:"
				print self.lisp_anom_str()
				self.MAAnomCount += 1
		self.gngAssess.run(verbose)
		
			