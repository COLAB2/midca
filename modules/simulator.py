import sys, random
from MIDCA import worldsim, goals
from MIDCA.customrun import customrun
from optparse import OptionParser


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
                self.customrun = customrun.CustomRun(self, tickFile)
                #self.tick_events = customrun.load_custom_run_xml(self,tickFile)

	def init(self, world, mem):
		self.mem = mem
		self.world = world
	

        def apply_change(self, change_str,verbose=2):
                try:
                        worldsim.stateread.apply_state_str(self.world, change_str)
                        if verbose >= 2:
                                print "Applying Change: ", change_str
                except Exception:
                        if verbose >= 1:
                                print "Failed Applying Change: ", change_str
                
                
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
	

        # here random refers to a random block, not a random fire - a
        # fire is gaurunteed to occur on a block unless all blocks are
        # on fire
        def start_random_fire(self,tempvar=0,tempvar2="default",verbose=2):
                print "tempvar is " + str(tempvar) + " and is of type " + str(type(tempvar))
                print "tempvar2 is " + tempvar2 + " and is of type " + str(type(tempvar2))
                change_str = "onfire()"
                try:
                        block = random.choice(self.get_unlit_blocks())
                        change_str = change_str[0:len(change_str)-1] # remove last paren
                        change_str += block + ")"
                        worldsim.stateread.apply_state_str(self.world, change_str)
                        if verbose >= 2:
                                print "Applying Change: ", change_str
                except Exception:
                        if verbose >= 1:
                                print "Failed Applying Change: ", change_str

        def start_fire(self,block,verbose=2):
                assert isinstance(block,str)
                change_str = "onfire("+str(block)+")" 
                try:
                        worldsim.stateread.apply_state_str(self.world, change_str)
                        if verbose >= 2:
                                print "Applying Change: ", change_str
                except Exception:
                        if verbose >= 1:
                                print "Failed Applying Change: ", change_str
                
        def add_fire_extinguisher(self,fire_ext_name,verbose=2):
                assert isinstance(fire_ext_name,str)
                change_str = "FIRE-EXTINGUISHER("+str(fire_ext_name)+")" 
                change_str2 = "fire-extinguisher("+str(fire_ext_name)+")" 
                try:
                        worldsim.stateread.apply_state_str(self.world, change_str)
                        if verbose >= 2:
                                print "Applying Change: ", change_str
                except Exception:
                        if verbose >= 1:
                                print "Failed Applying Change: ", change_str                
                try:
                        worldsim.stateread.apply_state_str(self.world, change_str2)
                        if verbose >= 2:
                                print "Applying Change: ", change_str2
                except Exception:
                        if verbose >= 1:
                                print "Failed Applying Change: ", change_str2                
              
        def remove_fire_extinguisher(self,fire_ext_name,verbose=2):
                assert isinstance(fire_ext_name,str)
                change_str = "!FIRE-EXTINGUISHER("+str(fire_ext_name)+")" 
                change_str2 = "!fire-extinguisher("+str(fire_ext_name)+")" 
                try:
                        worldsim.stateread.apply_state_str(self.world, change_str)
                        if verbose >= 2:
                                print "Applying Change: ", change_str
                except Exception:
                        if verbose >= 1:
                                print "Failed Applying Change: ", change_str                
                try:
                        worldsim.stateread.apply_state_str(self.world, change_str2)
                        if verbose >= 2:
                                print "Applying Change: ", change_str2
                except Exception:
                        if verbose >= 1:
                                print "Failed Applying Change: ", change_str2                


	def run(self, cycle, verbose = 2):
                print("Cycle is "+str(cycle))
                
                eval_stmts = self.customrun.run_events_on_cycle(cycle, self) 
                for eval_item in eval_stmts:
                        print "Now eval'ing item: "+str(eval_item)
                        eval(eval_item)

                
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
