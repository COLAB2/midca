import os
import sys
sys.path.append("../")
from domains.blocksworld.block import Block
from Tree_Fire.Tree import Tree as TreeFire
from Tree_3_Scen.Tree import Tree as TreeStack
from goals import Goal
from domains.blocksworld import blockstate
from XP_Goal.parser import *
from XP_Goal.traverser import *
from datetime import datetime

readSize = 100000

class TFFireGen:

    def __init__(self, mem, memKeys):
        self.tree = TreeFire()
        self.mem = mem
        self.memKeys = memKeys

    def gen_goals(self, verbose):
        world = self.mem.get(self.memKeys.MEM_STATES)[-1]
        blocks = blockstate.get_block_list(world)
        goal = self.tree.givegoal(blocks)
        if goal:
        	goal.priority = 1
        	return [goal]
        return []

class TFStackGen:

    def __init__(self, mem, memKeys):
        self.tree = TreeStack()
        self.mem = mem
        self.memKeys = memKeys

    def gen_goals(self, verbose):
        world = self.mem.get(self.memKeys.MEM_STATES)[-1]
        blocks = blockstate.get_block_list(world)
        return [self.tree.givegoal(blocks)]

class ExogenousGoalGen:

	def __init__(self, world):
		self.goals = self.predef_goals(world)

	#good across naming conventions, but true only for one start state.
	def predef_goals(self, world, iterations = 500):
		for object in world.objects:
			if world.is_true("on-table", [object]) and world.is_true("clear", [object]):
				C = Block(Block.SQUARE, object)
			elif world.is_true("on-table", [object]):
				A = Block(Block.SQUARE, object)
			elif world.is_true("clear", [object]):
				D = Block(Block.TRIANGLE, object)
			elif not world.is_true("table", [object]) and world.is_true("block", [object]):
				B = Block(Block.SQUARE, object)

		return [Goal(Goal.GOAL_ON, [C, B]), Goal(Goal.GOAL_ON, [D, C]), Goal(Goal.GOAL_ON, [D, B])] * iterations

	def gen_goals(self, verbose):
		if self.goals:
			return [self.goals.pop(0)]
		return []

class MAGoalGen:

	def __init__(self, mem, memKeys):
		self.memKeys = memKeys
		self.readSocket = mem.get(self.memKeys.SOCKET_R)

	def gen_goals(self, verbose):
		if self.readSocket:
			data = self.readSocket.recv(size)
			if verbose >= 2:
				print "This was given by Meta-AQUA: ", data

			pos1 = data.find("(BURNING (DOMAIN (VALUE ")
			pos2 = data.find("CONTROLS")
			if pos1 != -1:
				blockname = data[pos1+24:pos1+26]
				if verbose >= 2:
					print "The block to extinguish is: " + blockname
				block = None
				for b in blockset:
					if b.id == blockname:
						block = b
						break

				if block:
					return Goal(Goal.GOAL_NO_FIRE, [block])
				else:
					print "No such block!!!"
			elif pos2 != -1:
				return Goal(Goal.GOAL_APPREHEND, ["Gui Montag"])
		else:
			print "No socket open to read Meta-AQUA data from."

class FireGen:

	def __init__(self, mem, memKeys):
		self.mem = mem
		self.memKeys = memKeys

	def random_fired_block(self, world):
		res = []
		for objectname in world.objects:
			if world.is_true("onfire", [objectname]) and objectname != "table":
				res.append(world.objects[objectname])
		if not res:
			return None
		return random.choice(res)

	def gen_goals(self, verbose):
		world = self.mem.get(self.memKeys.MEM_STATES)[-1]
		block = self.random_fired_block(world)
		if not block:
			return []
		blocks = blockstate.get_block_list(world)
		for realblock in blocks:
			if realblock.id == block.name:
				block = realblock
				break
		goal = Goal(Goal.GOAL_NO_FIRE, [block])
		goal.priority = 1 #higher than stacking
		return [goal]

class ArsonistCatcher:

	def __init__(self, mem, memKeys):
		self.mem = mem
		self.memKeys = memKeys

	def free_arsonist(self):
		world = self.mem.get(self.memKeys.MEM_STATES)[-1]
		for atom in world.atoms:
			if atom.predicate.name == "free" and atom.args[0].type.name == "ARSONIST":
				return atom.args[0].name
		return False

	def gen_goals(self, verbose):
		arsonist = self.free_arsonist()
		anomalous = self.mem.get(self.memKeys.MEM_ANOM)
		anomalous = anomalous and anomalous[-1]
		if arsonist and anomalous:
			goal = Goal(Goal.GOAL_APPREHEND, [arsonist])
			goal.priority = 2 #highest used
			return [goal]
		return []

class XPGoalGen:

	bufferSize = 2000000000
	logfile = os.path.expanduser('~/meta-aqua-%s-output.log' % datetime.now().strftime('%Y-%m-%d'))

	def __init__(self, mem, memKeys):
		self.counter = 0
		self.mem = mem
		self.memKeys = memKeys

	def gen_goals(self, verbose):
		self.counter += 1
		if self.counter <= 1:
			return []

		world = self.mem.get(self.memKeys.MEM_STATES)[-1]
		blockset = blockstate.get_block_list(world)
		s = self.mem.get(self.memKeys.SOCKET_R)

		if verbose >= 2:
			print "In gengoal_fire.py"

		if s:
			text = s.recv(self.bufferSize)

			with open(self.logfile, 'a') as f:
				f.write(text)
				f.write("\n%s\n" % ("*"*75))

			if text != "None\n":
				if verbose >= 2:
					print "HERE IS TEXT: " + text

				# parse text
				p = Parser()
				frames = p.makeframegraph(text)

				# create mapping
				noem = {}   # Node Operator Effect Mapping
							# Keys are node/frame names, values are lists of [operatorname, effect] pairs

				noem['CRIMINAL-VOLITIONAL-AGENT.4697'] = [['apprehend', OPERATOR_EFFECT_NEGATION]]

				# Traverse
				t = Traverser(frames, noem)
				(frame, operator, effect) = t.traverse()

				if operator == "apprehend":
					return Goal(Goal.GOAL_APPREHEND, ["Gui Montag"])
				else:
					print "Unrecognized operator(!): " + str(operator) + ", now producing standard extinguish goal."
					for block in blockset:
						if block.onfire:
							return Goal(Goal.GOAL_NO_FIRE, [block])

