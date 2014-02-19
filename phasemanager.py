import copy, time, datetime
from mem import Memory

class Module:
	
	pass
	
class Phase:
	
	def __init__(self, name):
		self.name = name

class MIDCA:

	def __init__(self, world, simulator, options, memKeys, verbose = 2):
		self.world = world
		self.mem = Memory()
		self.memKeys = memKeys
		self.phases = []
		self.modules = {}
		self.add_module("Simulation", simulator)
		self.phasei = -1
		self.twoSevenWarning = False
		self.verbose = verbose
		self.options = options
	
	def phase_by_name(self, name):
		for phase in self.phases:
			if phase.name == name:
				return phase
		return None
	
	def insert_phase(self, phase, phaseOrIndex):
		if isinstance(phase, str):
			phase = Phase(phase)
		if not isinstance(phase, Phase):
			raise KeyError(str(phase) + " is not a valid phase or phase name.")
		if isinstance(phaseOrIndex, str):
			phaseOrIndex = self.phase_by_name(phaseOrIndex)
		elif isinstance(phaseOrIndex, int):
			self.phases.insert(phaseOrIndex, phase)
			return
		if not isinstance(phaseOrIndex, Phase):
			raise KeyError(str(phase) + " is not a valid phase or index.")
		if phaseOrIndex not in self.phases:
			raise KeyError("phase " + str(phaseOrIndex) + " not in phase list.")
		self.phases.insert(self.phases.index(phaseOrIndex), phase)
	
	def append_phase(self, phase):
		self.insert_phase(phase, len(self.phases) - 1)
	
	def assign_module(self, phase, module):
		if isinstance(phase, str):
			phase = self.phase_by_name(phase)
		if phase not in self.phases:
			raise KeyError("phase " + str(phaseOrIndex) + " not in phase list.")
		if not hasattr(module, "run"):
			raise AttributeError("All modules must a 'run' function")
		self.modules[phase] = module
	
	def add_module(self, phase, module):
		self.append_phase(phase)
		self.assign_module(phase, module)
	
	def get_module(self, phase):
		if isinstance(phase, str):
			phase = self.phase_by_name(phase)
		if phase in self.modules:
			return self.modules[phase]
		return None
	
	def init(self, verbose = 2):
		self.mem = Memory()
		for phase, module in self.modules.items():
			#try:
			if verbose >= 2:
				print "Initializing " + phase.name + " module...",
			module.init(self.world, self.mem, self.memKeys)
			print "done."
			'''
			except AttributeError:
				if verbose >= 2:
					print "\nPhase " + phase.name + " has no init function."
			except Exception as e:
				if verbose >= 1:
					print "\nPhase " + phase.name + " initialization failed."
				raise e
			'''
	
	def start(self, verbose = 1):
		if verbose >= 1:
			print "starting execution"
		self.phasei = 0
	
	def next_phase(self, verbose = 2):
		if self.phasei < 0:
			self.start()
		else:
			self.phasei = self.phasei % len(self.phases)
			if verbose >= 2:
				print "\n****** Starting", self.phases[self.phasei].name, "Phase ******\n"
			self.modules[self.phases[self.phasei]].run(verbose)
			self.phasei += 1
		
	
	def one_cycle(self, verbose = 1, pause = 0.5):
		for i in range(len(self.phases)):
			t1 = datetime.datetime.today()
			self.next_phase(verbose)
			t2 = datetime.datetime.today()
			try:
				if (t2 - t1).total_seconds() < pause:
					time.sleep(pause - (t2 - t1).total_seconds())
			except AttributeError:
				if not self.twoSevenWarning:
					print '\033[93m' + "Use python 2.7 or higher to get accurate pauses between steps. Continuing with approximate pauses." + '\033[0m'
					self.twoSevenWarning = True
				time.sleep(pause)
	
	def several_cycles(self, num, verbose = 1, pause = 0.01):
		for i in range(num):
			self.one_cycle(verbose, pause)
		
	

if __name__ == "__main__":
	l = [1, 2]
	l.insert(2, 3)
	print l