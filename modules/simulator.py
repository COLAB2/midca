import sys, random, worldsim

class MidcaActionSimulator:
	
	def init(self, world, mem):
		self.mem = mem
		self.world = world
	
	def run(self, cycle, verbose = 2):
		try:
			#get selected actions for this cycle. This is set in the act phase.
			actions = self.mem.get(self.mem.ACTIONS)[-1]
		except TypeError, IndexError:
			if verbose >= 1:
				print "Simulator: no actions selected yet by MIDCA."
			return
		if actions:
			for action in actions:
				if world.midca_action_applicable(action):
					if verbose >= 2:
						print "simulating MIDCA action:", action
					world.apply_midca_action(action)
				else:
					if verbose >= 1:
						print "MIDCA-selected action", action, "illegal in current world state. Skipping"
		else:
			if verbose >= 2:
				print "No actions selected this cycle by MIDCA."

ARSONIST_VICTORY_ACTIVITIES = ["enjoys a glass of champagne", "stays home", "bites his thumb at MIDCA"]

class ArsonSimulator:
	
	def __init__(self, arsonChance = 0.5, arsonStart = 10):
		self.chance = 0.5
		self.start = 10
	
	def init(self, world, mem):
		self.mem = mem
		self.world = world
	
	def free_arsonist(self):
		for atom in self.world.atoms:
			if atom.predicate.name == "free":
				if atom.args[0].type.name == "ARSONIST":
					return atom.args[0].name
		return False	
	
	def get_unlit_blocks(self):
		res = []
		for objectname in self.world.objects:
			if not self.world.is_true("onfire", [objectname]) and self.world.objects[objectname].type.name == "BLOCK" and objectname != "table":
				res.append(objectname)
		return res
	
	def run(self, cycle, verbose = 2):
		arsonist = self.free_arsonist()
		if arsonist and cycle > self.start and random.random() < self.chance:
			try:
				block = random.choice(self.get_unlit_blocks())
				try:
					self.world.apply_named_action("lightonfire", [arsonist, block])
					if verbose >= 2:
						print "Simulating action: lightonfire(" + str(arsonist) + ", " + str(block) + ")"
				except Exception:
					print "Action lightonfire(", str(arsonist), ",", str(block), ") invalid."
			except IndexError:
				print "All blocks on fire.", arsonist, random.choice(ARSONIST_VICTORY_ACTIVITIES)

	
