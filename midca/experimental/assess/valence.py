import sys
sys.path.append("../../")
from utils import adistadapt
from gngassess import *

'''
experimental MIDCA module that categorizes the type of an anomaly detected using the A-Distance. This is still in the early stages of development, though the module should run correctly so long as an A-Distance module is also incorporated into MIDCA.
'''

VALENCE_KEY = "valence"

class PredChange:
	
	def __init__(self, difs, fractiondifs, predname = None):
		self.difs = difs
		self.fractiondifs = fractiondifs
		self.predname = predname
	
	def __getitem__(self, val):
		return self.difs[val]

class PredValence:
	
	def __init__(self, pluses, nochange, minuses, predname = None):
		self.pluses = pluses
		self.nochange = nochange
		self.minuses = minuses
		self.predname = predname
	
	def compare(self, baseline):
		difs = [self.pluses - baseline.pluses, self.nochange - baseline.nochange, self.minuses - baseline.minuses]
		fractions = [float(difs[0]) / max(0.5, baseline.pluses), float(difs[1]) / max(0.5, baseline.nochange), float(difs[2]) / max(0.5, baseline.minuses)]
		return PredChange(difs, fractions, self.predname)		

class WindowChange:
	
	def __init__(self, predchanges, prednames = []):
		self.predchanges = predchanges
		self.changesByName = {}
		if prednames:
			for i in range(len(prednames)):
				self.changesByName[prednames[i]] = predchanges[i]
	
	def __getitem__(self, val):
		if val in self.changesByName:
			return self.changesByName[val]
		return self.predchanges[val]

class WindowValence:
	
	def __init__(self, pluses, nochange, minuses, preds = []):
		self.valences = []
		self.valencesByName = {}
		for i in range(len(pluses)):
			if preds:
				valence = PredValence(pluses[i], nochange[i], minuses[i], preds[i])
				self.valencesByName[preds[i]] = valence
			else:
				valence = PredValence(pluses[i], nochange[i], minuses[i])
			self.valences.append(valence)
	
	def compare(self, baseline):
		assert(len(self.valences) == len(baseline.valences))
		changes = [self.valences[i].compare(baseline.valences[i]) for i in range(len(self.valences))]
		return WindowChange(changes, [valence.predname for valence in self.valences if self.valencesByName])
	
	def __getitem__(self, val):
		if val in self.valencesByName:
			return self.valencesByName[val]
		return self.valences[val]
	
class PredicateChangeAssessor:

	CHANGE_INC = "change-increasing"
	CHANGE_DEC = "change-decreasing"
	POS_INC = "occurrence-increasing"
	NEG_INC = "occurrence-decreasing"
	
	def __init__(self, windowsize):
		self.size = windowsize
	
	def init(self, world, mem):
		self.mem = mem
		self.baseline = None
	
	#defaults to last window possible
	def window_valence(self, starti = None):
		states = self.mem.get(self.mem.STATES)
		if starti == None:
			starti = len(states) - self.size - 1
		if len(states) < self.size + starti + 1 or starti < 0: #window incomplete
			return
		lastcounts = adistadapt.get_count_vector(states[starti])
		numpreds = len(lastcounts)
		pluses = [0 for i in range(numpreds)]
		minuses = [0 for i in range(numpreds)]
		nochange = [0 for i in range(numpreds)]
		for i in range(starti + 1, starti + self.size + 1):
			newcounts = adistadapt.get_count_vector(states[i])
			for pred in range(len(lastcounts)):
				if newcounts[pred] > lastcounts[pred]:
					pluses[pred] += 1
				elif newcounts[pred] < lastcounts[pred]:
					minuses[pred] += 1
				else:
					nochange[pred] += 1
			lastcounts = newcounts
		prednames = self.mem.get(A_DIST_KEY)[0].trace.prednames
		return WindowValence(pluses, nochange, minuses, prednames)
	
	def build_baseline(self):
		self.baseline = self.window_valence(0)
	
	#add in positives decreasing, negatives decreasing
	def characterize_change(self, pred, valence, baseline):
		change = valence.compare(baseline)
		print change[pred][0], change[pred][1]
		if change[pred][0] >= abs(change[pred][1]):
			return self.POS_INC #number of predicate instances is increasing
		elif change[pred][2] >= abs(change[pred][1]):
			return self.NEG_INC #number of predicate instances is decreasing
		elif change[pred][1] < 0:
			return self.CHANGE_INC #variance in predicate instances is high
		else:
			return self.CHANGE_DEC #variance in predicate instance is low
	
	def run(self, cycle, verbose = 2):
		if not self.mem.get(ANOMALY_STATE_KEY) or not self.mem.get(ANOMALY_STATE_KEY)[-1]:
			if verbose >= 3:
				print "No valence analysis performed."
			return
		if not self.baseline:
			self.build_baseline()
		if not self.baseline:
			raise Exception("Cannot build baseline, though anomaly detected. Investigate.")
		valence = self.characterize_change(self.mem.get(ANOMALY_STATE_KEY)[-1], self.window_valence(), self.baseline)
		self.mem._update(VALENCE_KEY, valence)
		if verbose >= 1:
			print "anomaly valence: ", valence
		