import sys, os, collections
sys.path.append("../SHOP_2_7/GDP/trunk")
sys.path.append("../")
import runlisp
from utils import blockstate
import midca_inst
from plan.midcaplan import *

SHOP2_IN_F = "shop2in.lisp"
SHOP2_DOMAIN_F = "./examples/htn/blocks/domains/blocks-htn-fire.lisp"
SHOP2_OUT_F = "shop2out.lisp"

def shop2_str(blockset, goals):
	s = ""
	s += ";;-------------------------------\n"
	s += "(in-package :shop2)\n"
	s += ";;---------------------------------------------\n"
	s += "\n"
	s += "(defproblem p_some_problem blocks-htn\n"
	s += "(\n"
	s += "    (arm-empty)\n"

	for block in blockset:
		if block.type != block.TABLE:
			s += "    (block " + str(block.id) + ")\n"

	for block in blockset:
		if block.type != block.TABLE:
			if block.on == None:
				pass
			elif block.on.type == block.TABLE:
				s += "    (on-table " + str(block.id) + ")\n"
			else:
				s += "    (on " + str(block.id) + " " + str(block.on.id) + ")\n"

			if block.clear:
				s += "    (clear " + str(block.id) + ")\n"
		if not block.onfire:
				s += "    (notonfire " + str(block.id) + ")\n"

	s += ")\n"
	s += "(;Goal Task Network\n"
	s += "    (achieve-goals (\n"
	for goal in goals:
		s += "        " + goal.foil_str() + "\n"
	s += ")));End of goal task network\n"
	s += ")\n"
	return s

def parse_shop2_plan(filename):
	text = open(filename).read()
	if "--plan :" in text:
		if "(" not in text:
			print text
			return []
		planstart = text.index("(", text.index("--plan :"))
		planend = text.rindex(")")
		text = text[planstart + 1:planend]
		actions = []
		while "(" in text and ")" in text:
			action = text[text.index("(") + 1:text.index(")")].split()
			action[0] = action[0][1:]
			for i in range(len(action)):
				action[i] = action[i].lower()
			actions.append(action)
			text = text[text.index(")") + 1:]
		return Plan([Action(action[0], action[1:]) for action in actions])
	return Plan([])

class Shop2Planner:
	
	def init(self, world, mem):
		self.mem = mem
	
	#this will require a lot more error handling, but ignoring now for debugging.
	def run(self, verbose = 2):
		world = self.mem.get(midca_inst.MEM_STATES)[-1]
		blockset = blockstate.get_block_list(world)
		goals = self.mem.get(midca_inst.MEM_CUR_GOAL)
		if not goals:
			if verbose >= 2:
				print "No goal received by planner. Skipping planning."
			return
			
		if goals and not isinstance(goals, collections.Iterable):
			goals = [goals]
		
		allNone = True
		for goal in goals:
			if goal:
				allNone = False
				break
		if allNone:
			if verbose >= 2:
				print "No goal received by planner. Skipping planning."
			return
				
		#shop2
		if verbose >= 2:
			print "Planning...",
		shop2str = shop2_str(blockset, goals)
		infile = open(SHOP2_IN_F, "w")
		infile.write(shop2str)
		infile.close()
		runlisp.run_shop2(SHOP2_DOMAIN_F, SHOP2_IN_F, SHOP2_OUT_F)
		plan = parse_shop2_plan(SHOP2_OUT_F)
		if verbose >= 2:
			print "done. cleaning up files...",
		os.remove(SHOP2_IN_F)
		os.remove(SHOP2_IN_F[:-4] + "fasl")
		#os.remove(SHOP2_OUT_F)
		if verbose >= 2:
			print "done."
		#change from
		
		
		
		if verbose >= 1:
			print "Planning complete."
		if verbose >= 2:
			print "Plan: ",
			for step in plan:
				print str(step),
			print
		#save plan
		self.mem.add(midca_inst.MEM_PLANS, plan)
		self.mem.set(self.mem.CURR_PLAN, plan)
		