from midca.modules._robot_world import world_repr
from midca import rosrun, time
from midca.examples import ObjectDetector
from bzrlib.config import LocationStore

class ROSObserver:
	
	def init(self, world, mem):
		self.mem = mem
		self.mem.set(self.mem.STATE, world_repr.SimpleWorld())
	
	
		
	def run(self, cycle, verbose = 2):
		#self.ObserveWorld() 
		detectionEvents = self.mem.get_and_clear(self.mem.ROS_OBJS_DETECTED)
		detecttionBlockState = self.mem.get_and_clear(self.mem.ROS_OBJS_STATE)
		utteranceEvents = self.mem.get_and_clear(self.mem.ROS_WORDS_HEARD)
		feedback = self.mem.get_and_clear(self.mem.ROS_FEEDBACK)
		world = self.mem.get_and_lock(self.mem.STATE)
		
		if not detectionEvents:
			detectionEvents = []
		if not detecttionBlockState:
			detecttionBlockState = []
		if not utteranceEvents:
			utteranceEvents = []
		if not feedback:
			feedback = []
		for event in detectionEvents:
			event.time = time.now()
			world.sighting(event)
		for blockstate in detecttionBlockState:
			blockstate.time = time.now()
			world.position(blockstate)
		for event in utteranceEvents:
			event.time = time.now()
			world.utterance(event)
		for msg in feedback:
			d = rosrun.msg_as_dict(msg)
			d['received_at'] = float(time.now())
			self.mem.add(self.mem.FEEDBACK, d)
		self.mem.unlock(self.mem.STATE)
		if verbose > 1:
			print "World observed:", len(detectionEvents), "new detection event(s),", len(utteranceEvents), "utterance(s) and", len(feedback), "feedback msg(s)"
			
	

class PerfectObserver:

	'''
	MIDCA Module which copies a complete world state. It is designed to interact with the 
	built-in MIDCA world simulator. To extend this to work with other representations, 
	modify the observe method so that it returns an object representing the current known 
	world state.
	'''

	def init(self, world, mem):
		self.world = world
		self.mem = mem
	
	#perfect observation
	def observe(self):
		return self.world.copy()
		
	def run(self, cycle, verbose = 2):
		world = self.observe()
		if not world:
			raise Exception("World obervation failed.")
		self.mem.add(self.mem.STATES, world)
		if verbose >= 1:
			print "World observed."

class MAReport:
	
	namecounts = {"report": 0}
	
	def __init__(self):
		self.actions = []
		self.finalstate = None
	
	def str_dict(self, item, numtabs = 1, skipfirsttab = True):
		if isinstance(item, dict):
			s = ""
			first = True
			for key in sorted(item.keys()):
				if not first or not skipfirsttab:
					s += "\t" * numtabs
				else:
					s += " "
				s += "(" + str(key) + " " + self.str_dict(item[key], numtabs + 1) + ")\n"
				first = False
			s = s[:-1]
		else:
			s = str(item)
		return s
	
	def action_str(self, action):
		if action[0] in self.namecounts:
			self.namecounts[action[0]] += 1
		else:
			self.namecounts[action[0]] = 1
		s = "(" + str(action[0]) + "." + str(self.namecounts[action[0]]) + "\n"
		valuepairs = {}
		if action[0] in ("stack", "unstack", "pickup", "putdown", "apprehend", "putoutfire"):
			valuepairs["actor"] = {"value": "person.0"}
		elif action[0] == "burns":
			valuepairs["actor"] = {"value": "nature"}
		valuepairs["object"] = {"value": str(action[1]).replace(" ", "_")}
		if action[0] in ("stack", "unstack"):
			valuepairs["recipient"] = {"value": str(action[2]).replace(" ", "_")}
		return s + self.str_dict(valuepairs, skipfirsttab = False) + ")"
	
	def atom_pairs(self, atom):
		valuepairs = {}
		for i in range(len(atom.args)):
			if i == 0:
				valuepairs["domain"] = {"value": atom.args[i].name.replace(" ", "_")}
			elif i == 1:
				valuepairs["co-domain"] = {"value": atom.args[i].name.replace(" ", "_")}
		return valuepairs
	
	def state_str(self, world):
		s = "(state\n"
		valuepairs = {}
		for atom in world.atoms:
			if atom.predicate.name in self.namecounts:
				self.namecounts[atom.predicate.name] += 1
			else:
				self.namecounts[atom.predicate.name] = 1
			valuepairs[atom.predicate.name + "." + str(self.namecounts[atom.predicate.name])] = self.atom_pairs(atom)
		return s + self.str_dict(valuepairs, skipfirsttab = False) + ")"
		
	
	def __str__(self):
		if not self.finalstate:
			return "incomplete"
		else:
			self.namecounts["report"] += 1
			s = "(" + "report." + str(self.namecounts["report"]) + "\n("
			for action in self.actions:
				s += "\t(\n"
				s += self.action_str(action)
				s += "\n\"\")\n("
				s += self.state_str(self.finalstate)
				s += "\n\"\")\n"
			return s + "))"

'''
ma = MAReport()
import domainread, stateread
world = domainread.load_domain("./domain.sim")
stateread.apply_state_file(world, "./defstate.sim")
ma.finalstate = world
ma.actions.append(["unstack", "block1", "block2"])
ma.actions.append(["catchfire", "block1"])
print ma
'''

class MAReporter:
	
	'''
	MIDCA module that sends a report on the world and actions to the Meta-AQUA story understanding system. This requires Meta-AQUA to be running or it will not work. Also depends on the basic observation module.
	'''
	
	def init(self, world, mem):
		self.mem = mem
	
	def get_lit_blocks(self, world):
		res = []
		for objectname in world.objects:
			if world.is_true("onfire", [objectname]) and world.objects[objectname].type.name == "BLOCK" and objectname != "table":
				res.append(objectname)
		return res
	
	def run(self, cycle, verbose = 2):
		world = None
		lastWorld = None
		try:
			world = self.mem.get(self.mem.STATES)[-1]
			lastWorld = self.mem.get(self.mem.STATES)[-2]
		except TypeError, IndexError:
			pass #leave as None
		if not world:	
			return #no report if not world observed
		report = MAReport()
		report.finalState = world
		try:
			actions = self.mem.get(self.memKeys.MEM_ACTIONS)[-1]
		except TypeError, IndexError:
			actions = []
		blocksPutOut = []
		for action in actions:
			report.actions.append([action.op.name] + action.args)
			if action.op.name == "putoutfire":
				blocksPutOut.append(action.args[0])
		if lastWorld:
			lastBurning = self.get_lit_blocks(lastWorld)
			burning = self.get_lit_blocks(world)
			for block in burning:
				if block not in lastBurning or block in blocksPutOut:
					report.actions.append(["burns", block])
		#report is finished, send to Meta-AQUA
			
