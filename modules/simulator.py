import sys, random
from MIDCA import worldsim, goals

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
				if self.world.midca_action_applicable(action):
					if verbose >= 2:
						print "simulating MIDCA action:", action
					self.world.apply_midca_action(action)
				else:
					if verbose >= 1:
						print "MIDCA-selected action", action, "illegal in current world state. Skipping"
		else:
			if verbose >= 2:
				print "No actions selected this cycle by MIDCA."

ARSONIST_VICTORY_ACTIVITIES = ["enjoys a glass of champagne", "stays home", "bites his thumb at MIDCA"]

from MIDCA.worldsim import blockstate, scene

class ASCIIWorldViewer:
	
	def init(self, world, mem):
		self.world = world
	
	def run(self, cycle, verbose = 2):
		if verbose >= 2:
			blocks = blockstate.get_block_list(self.world)
			print str(scene.Scene(blocks))

class WorldChanger:
	
	def init(self, world, mem):
		self.world = world
	
	def parseGoal(self, txt):
		if not txt.endswith(")"):
			print "Error reading input. Atom must be given in the form: predicate(arg1, arg2,...,argi-1,argi), where each argument is the name of an object in the world"
			return None
		try:
			predicateName = txt[:txt.index("(")]
			args = [arg.strip() for arg in txt[txt.index("(") + 1:-1].split(",")]
			goal = goals.Goal(*args, predicate = predicateName)
			return goal
		except Exception:
			print "Error reading input. Atom must be given in the form: predicate(arg1, arg2,...,argi-1,argi), where each argument is the name of an object in the world"
			return None
	
	def run(self, cycle, verbose = 2):
		if verbose == 0:
			return
		while True:
			val = raw_input("If you wish to change the state, please input the desired atom to flip. Otherwise, press enter to continue\n")
			if not val:
				return "continue"
			elif val == 'q':
				return val
			goal = self.parseGoal(val.strip())
			if goal:
				try:
					atom = self.world.midcaGoalAsAtom(goal)
					if self.world.atom_true(atom):
						self.world.remove_atom(atom)
						print "Atom", atom, "was true and is now false"
					else:
						self.world.add_atom(atom)
						print "Atom", atom, "was false and is now true"
				except ValueError:
					 "The value entered does not appear to be a valid atom. Please check the number and type of arguments."
					
class ArsonSimulator:
	
	def __init__(self, arsonChance = 0.5, arsonStart = 10):
		self.chance = arsonChance
		self.start = arsonStart
	
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
					if verbose >= 1:
						print "Action lightonfire(", str(arsonist), ",", str(block), ") invalid."
			except IndexError:
				if verbose >= 1:
					print "All blocks on fire.", arsonist, random.choice(ARSONIST_VICTORY_ACTIVITIES)

SCORE = "Score"

class FireReset:
	
	'''
	MIDCA module that puts out all fires whenever MIDCA's score is updated to indicate that a tower has been completed. Note that this module will do nothing unless the the SCORE memory value is being updated by evaluate.Scorer.
	'''
	
	def init(self, world, mem):
		self.world = world
		self.mem = mem
		self.numTowers = 0
	
	def put_out_fires(self):
		self.world.atoms = [atom for atom in self.world.atoms if atom.predicate.name != "onfire"]
	
	def run(self, cycle, verbose = 2):
		score = self.mem.get(SCORE)
		if not score:
			return
		if score.towers == self.numTowers:
			return
		self.numTowers = score.towers
		if verbose >= 2:
			print "Since a tower was just completed, putting out all fires."
		self.put_out_fires()