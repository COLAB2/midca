import sys, random
from MIDCA import worldsim, goals

## Tick File Code (not sure if I should make a seperate module)
from optparse import OptionParser
import inspect
import xml.etree.ElementTree as ET

def load_tick_file_csv(calling_instance, file_name, verbose = 2):
        """Reads in a tick file from a csv and assumes all functions take no
           arguments (if you want to use arguments to functions, use
           the alternative function: load_tick_file_xml)

        """
        valid_methods = []
        for data in inspect.getmembers(calling_instance, predicate=inspect.ismethod):
                #print "data is " + str(data)
                if not data[0].startswith('__'): # ignore class only methods
                        valid_methods.append(data[0])
        
        # a tick file is stored as a dictionary where keys are
        # integers representing current 'round' of the
        # simulation and the values are an array of functions
        # to be executed, in the order they are given from
        # left to right. These functions must be defined in
        # the arsonist class that will use the tick file.
        tick_events = {}
        with open(file_name) as f:
                lines = f.readlines()
                for line in lines:
                        tokens = line.strip().split(',')
                        curr_tick = int(tokens[0])
                        tick_events[curr_tick] = tokens[1:]
                        if verbose >= 2: print "processed line: " + line
                        print "self.tick_events: " + str(tick_events)

        return tick_events

def load_tick_file_xml(calling_instance, file_name, verbose = 2):
        valid_methods = []
        for data in inspect.getmembers(calling_instance, predicate=inspect.ismethod):
                #print "data is " + str(data)
                if not data[0].startswith('__'): # ignore class only methods
                        valid_methods.append(data[0])
        
        # a tick file is stored as a dictionary where keys are
        # integers representing current 'round' of the
        # simulation and the values are an array of functions
        # to be executed, in the order they are given from
        # left to right. These functions must be defined in
        # the arsonist class that will use the tick file.
        tick_events = {}
        
        
        
        with open(file_name) as f:
                lines = f.readlines()
                for line in lines:
                        tokens = line.strip().split(',')
                        curr_tick = int(tokens[0])
                        tick_events[curr_tick] = tokens[1:]
                        if verbose >= 2: print "processed line: " + line
                        print "self.tick_events: " + str(tick_events)

        return tick_events

## End Tick File 

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
	
	def __init__(self, arsonChance = 0.5, arsonStart = 10, tickFile = False):
		self.chance = arsonChance
		self.start = arsonStart
                if tickFile:
                        self.tick_events = load_tick_file(self,tickFile)

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
                print("Cycle is "+str(cycle))
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

        def start_random_fire():
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

        def add_fire_extinguisher():
        def remove_fire_extinguisher():

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
