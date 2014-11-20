import copy, time, datetime
from mem import Memory

MAX_MODULES_PER_PHASE = 100
	
class Phase:
	
	def __init__(self, name):
		self.name = name
	
	def __str__(self):
		return self.name
	
	def __eq__(self, other):
		return type(self) == type(other) and self.name == other.name
	
	def __hash__(self):
		return hash(self.name)

class MIDCA:

	def __init__(self, world, verbose = 2):
		self.world = world
		self.mem = Memory()
		self.phases = []
		self.modules = {}
		self.phaseNum = 1
		self.twoSevenWarning = False
		self.verbose = verbose
		self.displayFunction = None
		self.initialized = False
	
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
			self.modules[phase] = []
			return
		if not isinstance(phaseOrIndex, Phase):
			raise KeyError(str(phase) + " is not a valid phase or index.")
		if phaseOrIndex not in self.phases:
			raise KeyError("phase " + str(phaseOrIndex) + " not in phase list.")
		self.phases.insert(self.phases.index(phaseOrIndex), phase)
		self.modules[phase] = []
	
	def append_phase(self, phase):
		self.insert_phase(phase, len(self.phases) + 1)
	
	def append_module(self, phase, module):
		self.insert_module(phase, module, MAX_MODULES_PER_PHASE)
	
	def insert_module(self, phase, module, i):
		if isinstance(phase, str):
			phase = self.phase_by_name(phase)
		if phase not in self.phases:
			raise KeyError("phase " + str(phase) + " not in phase list. Call insert_phase() or append_phase() to add it.")
		if not hasattr(module, "run"):
			raise AttributeError("All modules must a 'run' function")
		if len(self.modules[phase]) == MAX_MODULES_PER_PHASE:
			raise Exception("max module per phase [" + str(MAX_MODULES_PER_PHASE) + "] exceeded for phase" + str(phase) + ". Cannot add another.")
		self.modules[phase].insert(i, module)
	
	def clearPhase(self, phase):
		self.modules[phase] = []
	
	def get_modules(self, phase):
		if isinstance(phase, str):
			phase = self.phase_by_name(phase)
		if phase in self.modules:
			return self.modules[phase]
		else:
			raise ValueError("No such phase as " + str(phase))
	
	def init(self, verbose = 2):
		self.mem = Memory()
		for phase, modules in self.modules.items():
			i = 0
			for module in modules:
				i += 1
				try:
					if verbose >= 2:
						print "Initializing " + phase.name + " module " + str(i) + "...",
					module.init(self.world, self.mem)
					print "done."
				
				except AttributeError:
					if verbose >= 2:
						print "\nPhase " + phase.name + " module " + str(i) +  "has no init function. Skipping init."
				except Exception as e:
					if verbose >= 1:
						print "\nPhase " + phase.name + " module " + str(i) + "initialization failed."
					raise e
		self.initialized = True
	
	def next_phase(self, verbose = 2):
		self.phasei = (self.phaseNum - 1) % len(self.phases)
		if verbose >= 2:
			print "\n****** Starting", self.phases[self.phasei].name, "Phase ******\n"
		for module in self.modules[self.phases[self.phasei]]:
			module.run((self.phaseNum - 1) / len(self.phases), verbose)
		if self.phasei == 0 and self.displayFunction:
			self.displayFunction(self.world)
		self.phaseNum += 1
		
	
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
	
	#MIDCA will call this function after the first phase. The function should take one input, which will be whatever is stored in self.world.
	def set_display_function(self, function):
		self.displayFunction = function
	
	def run(self):
		if not self.initialized:
			raise Exception("MIDCA has not been initialized! Please call Midca.init() before running.")
		while 1:
			print "MIDCA is starting. Please enter commands, or '?' + enter for help. Pressing enter with no input will advance the simulation by one phase."
			val = raw_input()
			if val == "q":
				break
			elif val == "skip":
				self.one_cycle(verbose = 0, pause = 0)
				print "cycle finished"
			elif val == "show":
				if self.displayFunction:
					self.displayFunction(self.world)
				else:
					print "No display function set. See Midca.set_display_function()"				
			elif val.startswith("skip"):
				try:
					num = int(val[4:].strip())
					for i in range(num):
						self.one_cycle(verbose = 0, pause = 0)
					print str(num) + " cycles finished."
				except ValueError:
					print "Usage: 'skip n', where n is an integer"
			elif val == "?" or val == "help":
				print "interface: \n enter/return -> input commands. Empty command goes to next cycle \n q -> quit \n skip n -> skips n cycles \n show -> print world representation \n ? or help -> show this list of commands \n"
			else:
				self.next_phase()
		print "MIDCA is quitting."
	

if __name__ == "__main__":
	l = [1, 2]
	l.insert(2, 3)
	print l