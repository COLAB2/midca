from __future__ import print_function
import copy, time, datetime, sys
from MIDCA.mem import Memory
from MIDCA import goals, logging
from MIDCA.worldsim import stateread

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
		self.verbose = verbose
		self.initialized = False
		self.phaseNum = 1
		self.logger = logging.Logger()
	
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
	
	def remove_phase(self, phaseOrName):
		if isinstance(phaseOrName, str):
			phase = self.phase_by_name(phaseOrName)
		else:
			phase = phaseOrName
		try:
			self.phases.remove(phase)
			del self.modules[phase]
		except ValueError:
			raise ValueError("Phase " + str(phaseOrName) + " is not a phase.")
		#if there is a KeyError, something has gone very wrong.
	
	def append_module(self, phase, module):
		self.insert_module(phase, module, MAX_MODULES_PER_PHASE)
	
	#note: error handling should be cleaned up - if a phase cannot be found by name, the error will report the phase name as "None" instead of whatever was given. True for removeModule as well.
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
	
	def removeModule(self, phase, i):
		if isinstance(phase, str):
			phase = self.phase_by_name(phase)
		if phase not in self.modules:
			raise KeyError("phase " + str(phase) + " not in phase list. Call insert_phase() or append_phase() to add it.")
		modules = self.modules[phase]
		if i < 0 or i >= len(modules):
			raise IndexError("index " + str(i) + " is outside the range of the module list for phase " + str(phase))
		else:
			modules.pop(i)
		
	
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
		for phase in self.phases:
			modules = self.modules[phase]
			i = 0
			for module in modules:
				i += 1
				try:
					if verbose >= 2:
						print("Initializing " + phase.name + " module " + str(i) + "...",)
					module.init(self.world, self.mem)
					print("done.")
				
				except Exception as e:
					print(e)
					if verbose >= 2:
						print("\nPhase " + phase.name + " module " + str(i) +  "has no init function or had an error. Skipping init.")
		self.initGoalGraph(overwrite = False)
		self.initialized = True
	
	def initGoalGraph(self, cmpFunc = None, overwrite = True):
		if overwrite or not self.mem.get(self.mem.GOAL_GRAPH):
			self.mem.set(self.mem.GOAL_GRAPH, goals.GoalGraph(cmpFunc))
			print("Goal Graph initialized.",)
			if cmpFunc:
				print()
			else:
				print("To use goal ordering, call initGoalGraph manually with a custom goal comparator")
	
	def next_phase(self, verbose = 2):
		retVal = ""
		self.phasei = (self.phaseNum - 1) % len(self.phases)
		if self.phasei == 0:
			self.logger.logEvent(logging.CycleStartEvent((self.phaseNum - 1) / len(self.phases)))
		if verbose >= 2:
			print("****** Starting", self.phases[self.phasei].name, "Phase ******\n", file = sys.stderr)
			self.logger.logEvent(logging.PhaseStartEvent(self.phases[self.phasei].name))
		for module in self.modules[self.phases[self.phasei]]:
			self.logger.logEvent(logging.ModuleStartEvent(module))
			retVal = module.run((self.phaseNum - 1) / len(self.phases), verbose)
			self.logger.logEvent(logging.ModuleEndEvent(module))
		self.logger.logEvent(logging.PhaseEndEvent(self.phases[self.phasei].name))
		self.phaseNum += 1
		if (self.phaseNum - 1) % len(self.phases) == 0:
			self.logger.logEvent(logging.CycleEndEvent((self.phaseNum - 1) / len(self.phases)))
		return retVal
	
class PhaseManager:
	
	def __init__(self, world, verbose = 2, display = None, storeHistory = False):
		self.midca = MIDCA(world, verbose)
		self.mem = self.midca.mem
		self.storeHistory = storeHistory
		self.history = []
		self.display = display
		self.twoSevenWarning = False
		self.logger = self.midca.logger
	
	'''
	convenience functions which wrap MIDCA functions
	'''
	def phase_by_name(self, name):
		return self.midca.phase_by_name(name)
	
	def insert_phase(self, phase, phaseOrIndex):
		self.midca.insert_phase(phase, phaseOrIndex)
	
	def append_phase(self, phase):
		self.midca.append_phase(phase)
	
	def get_phases(self):
		return [phase.name for phase in self.midca.phases]
	
	def append_module(self, phase, module):
		self.midca.append_module(phase, module)
	
	def insert_module(self, phase, module, i):
		self.midca.insert_module(phase, module, i)
	
	def remove_module(self, phase, i):
		self.midca.removeModule(phase, i)
	
	def clear_phase(self, phase):
		self.midca.clearPhase(phase)
	
	def get_modules(self, phase):
		return self.midca.get_modules(phase)
	
	def init(self, verbose = 2):
		self.midca.init(verbose)
	
	def initGoalGraph(self, cmpFunc = None):
		self.midca.initGoalGraph(cmpFunc)
	'''
	functions for advancing through phases and complete cycles.
	'''

	def next_phase(self, verbose = 2):
		if self.storeHistory:
			self.history.append(copy.deepcopy(self.midca))
		val = self.midca.next_phase(verbose)
		return val
	
	def one_cycle(self, verbose = 1, pause = 0.5):
		for i in range(len(self.midca.phases)):
			t1 = datetime.datetime.today()
			self.next_phase(verbose)
			t2 = datetime.datetime.today()
			try:
				if (t2 - t1).total_seconds() < pause:
					time.sleep(pause - (t2 - t1).total_seconds())
			except AttributeError:
				if not self.twoSevenWarning:
					print('\033[93m' + "Use python 2.7 or higher to get accurate pauses between steps. Continuing with approximate pauses." + '\033[0m')
					self.twoSevenWarning = True
				time.sleep(pause)
	
	def several_cycles(self, num, verbose = 1, pause = 0.01):
		for i in range(num):
			self.one_cycle(verbose, pause)
	
	#MIDCA will call this function after the first phase. The function should take one input, which will be whatever is stored in self.midca.world.
	def set_display_function(self, function):
		self.display = function

	def clearWorldState(self):
		self.midca.world.objects = {}
		self.midca.world.atoms = []
	
	def applyStateChange(self, stateStr):
		stateread.apply_state_str(self.midca.world, stateStr)

	#function which runs MIDCA with a text UI
	def run(self):
		if not self.midca.initialized:
			raise Exception("MIDCA has not been initialized! Please call Midca.init() before running.")
		print("\nMIDCA is starting. Please enter commands, or '?' + enter for help. Pressing enter with no input will advance the simulation by one phase.")
		while 1:
			print("Next MIDCA command:  ", file = sys.stderr, end = "")
			val = raw_input()
			print
			if val == "q":
				break
			elif val == "skip":
				self.one_cycle(verbose = 0, pause = 0)
				print("cycle finished")
			elif val == "show":
				if self.display:
					try:
						self.display(self.midca.world)
					except Exception as e:
						print("Error displaying world")
				else:
					print("No display function set. See PhaseManager.set_display_function()"	)			
			elif val.startswith("skip"):
				try:
					num = int(val[4:].strip())
					for i in range(num):
						self.one_cycle(verbose = 0, pause = 0)
					print(str(num) + " cycles finished.")
				except ValueError:
					print("Usage: 'skip n', where n is an integer")
			elif val == "log":
				print("Input the text to add to MIDCA's log file. Leave empty and press enter to cancel\n", file = sys.stderr)
				txt = raw_input()
				if txt:
					self.logger.log(txt)
			elif val == "change":
				print("Enter 'clear' to clear the world state, 'file' to input a state file name, or nothing to finish. Otherwise, enter changes to the world state. Use ! to negate atoms or remove objects, e.g. !on(A,B). Note that syntax is shared with state files in midca/worldsim/states, and each command must be on it's own line.")
				while True:
					input = raw_input("Next change:  ")
					if not input:
						break
					elif input == "clear":
						self.clearWorldState()
						print("World state cleared")
					elif input == "file":
						print("Enter the name of a valid state file, or leave blank to cancel.")
						filename = raw_input()
						if filename == "":
							print("File load cancelled")
							continue
						s = ""
						try:
							s = open(filename).read()
						except IOError:
							print("Cannot open file")
						try:
							self.applyStateChange(s)
							print("State loaded")
						except exception as e:
							print("Error loading state. State may be partially loaded: ", str(e))
					else:
						try:
							self.applyStateChange(input)
							print("Change applied")
						except Exception as e:
							print(e)
			elif val == "?" or val == "help":
				print("interface: \n enter/return -> input commands. Empty command goes to next cycle \n q -> quit \n skip n -> skips n cycles \n show -> print world representation \n change -> modify or clear world state \n log -> log some text \n ? or help -> show this list of commands \n")
			elif val:
				print("command not understood")
			else:
				val = self.next_phase()
				if val == "continue":
					self.next_phase()
				elif val == "q":
					break
		print("MIDCA is quitting.")
