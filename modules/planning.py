from _plan import pyhop, methods, operators, methods_extinguish, operators_extinguish
from MIDCA import plans
import collections
import traceback	
from MIDCA.modules._plan.asynch import asynch, methods, monitors

class GenericPyhopPlanner:
	
	'''
	Whereas the PyHopPlanner class below is optimized for use with MIDCA's built-in world
	simulator, this planner is more generalized. It assumes that the world state stored
	in MIDCA's memory is also the world state that will be expected by the planning 
	methods and operators. Also, it expects to receive 'declare_methods' and
	'declare_operators' methods as arguments. These should initialize pyhop for the 
	desired planning domain. The plan_validator arg should be a method which takes a
	world state and a plan as args and returns whether the plan should be used. This will
	only be called on old plans that are retrieved.
	'''
	
	def __init__(self, declare_methods, declare_operators, declare_monitors, plan_validator = None):
		try:
			declare_methods()
			declare_operators()
			declare_monitors()
			self.working = True
		except:
			print "Error declaring pyhop methods and operators. This planner will be \
			disabled"
			traceback.print_exc()
			self.working = False
		if plan_validator:
			self.validate_plan = plan_validator
		else:
			self.validate_plan = lambda state, plan: asynch.FAILED not in [action.status for action in plan]
			#note by default plans execute to completion unless goals 
			#change or an action fails.
	
	def init(self, world, mem):
		self.mem = mem
	
	def get_old_plan(self, state, goals, verbose = 2):
		try:
			plan = self.mem.get(self.mem.GOAL_GRAPH).getMatchingPlan(goals)
			if not plan:
				return None
			try:
				valid = self.validate_plan(state, plan)
				if verbose >= 2:
					if valid:
						print "Old plan found that tests as valid:", plan
					else:
						print "Old plan found that tests as invalid:", plan, ". removing from stored plans."
						self.mem.get(self.mem.GOAL_GRAPH).removePlan(plan)
			except:
				if verbose >= 2:
					print "Error validating plan:", plan
				valid = False
		except:
			print "Error checking for old plans"
			plan = None
			valid = False
		if valid:
			return plan
		return None
	
	def get_new_plan(self, state, goals, verbose = 2):
		if verbose >= 2:
			print "Planning..."
		try:
			plan = pyhop.pyhop(state, [("achieve_goals", goals)], verbose = 0)
			#note: MIDCA does not convert its state and goals to pyhop state and
			#goal objects. Therefore, pyhop will not print correctly if verbose is
			#set to other than 0.
		except:
			if verbose >= 1:
				print "Error in planning:", traceback.format_exc(), "\n-Planning failed."
			return None
		return plan
	
	def run(self, cycle, verbose = 2):
		state = self.mem.get(self.mem.STATE)
		if not state:
			states  = self.mem.get(self.mem.STATES)
			if states:
				state = states[-1]
			else:
				if verbose >= 1:
					print "No world state loaded. Skipping planning."
				return
		#now state is the most recent (or only) state and is non-null
		
		goals = self.mem.get(self.mem.CURRENT_GOALS)
		if not goals:
			if verbose >= 2:
				print "No goals received by planner. Skipping planning."
			return
		plan = self.get_old_plan(state, goals, verbose)
		if verbose > 2:
			if plan:
				print "Will not replan"
			else:
				print "Will replan"	
		if not plan:
			plan = self.get_old_plan(state, goals, verbose)
		if not plan:
			return
		midcaPlan = plans.Plan(plan, goals) 
		if verbose >= 1:
			print "Planning complete."
			if verbose >= 2:
				print "Plan: ", midcaPlan
		#save new plan
		if midcaPlan != None:
			self.mem.get(self.mem.GOAL_GRAPH).addPlan(midcaPlan)

class AsynchPyhopPlanner(GenericPyhopPlanner):
	
	'''
	This planner is the same as the GenericPyhopPlanner, but it returns an asynchronous
	plan.
	'''
	
	def run(self, cycle, verbose = 2):
		state = self.mem.get(self.mem.STATE)
		if not state:
			states  = self.mem.get(self.mem.STATES)
			if states:
				state = states[-1]
			else:
				if verbose >= 1:
					print "No world state loaded. Skipping planning."
				return
		#now state is the most recent (or only) state and is non-null
		
		goals = self.mem.get(self.mem.CURRENT_GOALS)
		if not goals:
			if verbose >= 2:
				print "No goals received by planner. Skipping planning."
			return
		plan = self.get_old_plan(state, goals, verbose)
		if verbose > 2:
			if plan:
				print "Will not replan"
			else:
				print "Will replan"	
		if plan:
			return
		if not plan:
			plan = self.get_new_plan(state, goals, verbose)
		if not plan:
			return
		midcaPlan = plans.Plan(plan, goals)
		asynchPlan = asynch.asynch_plan(self.mem, midcaPlan)
		if verbose >= 1:
			print "Planning complete."
			if verbose >= 2:
				print "Plan: ", asynchPlan
		#save new plan
		if asynchPlan != None:
			self.mem.get(self.mem.GOAL_GRAPH).addPlan(asynchPlan)
	
class PyHopPlanner:
	
	'''
	MIDCA module that implements a python version of the SHOP hierarchical task network (HTN) planner. HTN planners require a set of user-defined methods to generate plans; these are defined in the methods python module and declared in the constructor for this class.
	Note that this module uses has several methods to translate between MIDCA's world and goal representations and those used by pyhop; these should be changed if a new domain is introduced.
	'''
	
	def __init__(self, extinguishers = False):
		#declares pyhop methods. This is where the planner should be given the domain information it needs.
		if extinguishers:
			methods_extinguish.declare_methods()
			operators_extinguish.declare_ops()
			monitors.declare_monitors
		else:
			methods.declare_methods()
			operators.declare_ops()
			monitors.declare_monitors
	
	def init(self, world, mem):
		self.mem = mem
		#load operators from world. Note that using this simple method MIDCA will not check the types or values of arguments, though it will check for plan validity.
		#Also, this method depends on the default MIDCA world simulator.
		self.operators = {op.name: plans.Operator(op.name, op.objnames) for op in world.operators.values()}
	
	#this will require a lot more error handling, but ignoring now for debugging.
	def run(self, cycle, verbose = 2):
		world = self.mem.get(self.mem.STATES)[-1]
		goals = self.mem.get(self.mem.CURRENT_GOALS)
		if not goals:
			if verbose >= 2:
				print "No goals received by planner. Skipping planning."
			return
		try:
			midcaPlan = self.mem.get(self.mem.GOAL_GRAPH).getMatchingPlan(goals)
		except AttributeError:
			midcaPlan = None
		if midcaPlan:
			if verbose >= 2:
				print "Old plan retrieved. Checking validity...",
			valid = world.plan_correct(midcaPlan)
			if not valid:
				midcaPlan = None 
				#if plan modification is added to MIDCA, do it here.
				if verbose >= 2:
					print "invalid."
			elif verbose >= 2:
				print "valid."
			if valid:
				if verbose >= 2:
					print "checking to see if all goals are achieved...",
				achieved = world.plan_goals_achieved(midcaPlan)
				if verbose >= 2:
					if len(achieved) == len(midcaPlan.goals):
						print "yes"
					else:
						print "no. Goals achieved: " + str({str(goal) for goal in achieved})
				if len(achieved) != len(midcaPlan.goals):
					midcaPlan = None #triggers replanning.
		
		#ensure goals is a collection to simplify things later.
		if not isinstance(goals, collections.Iterable):
			goals = [goals]
		
		if not midcaPlan:
			#use pyhop to generate new plan
			if verbose >= 2:
				print "Planning..."
			try:
				pyhopState = self.pyhop_state_from_world(world)
			except Exception:
				print "Could not generate a valid pyhop state from current world state. Skipping planning"
			try:
				pyhopTasks = self.pyhop_tasks_from_goals(goals)
			except Exception:
				print "Could not generate a valid pyhop task from current goal set. Skipping planning"
			try:
				pyhopPlan = pyhop.pyhop(pyhopState, pyhopTasks, verbose = 0)
			except Exception:
				pyhopPlan = None
			if not pyhopPlan and pyhopPlan != []:
				if verbose >= 1:
					print "Planning failed for ",
					for goal in goals:
						print goal, " ",
					print
				return
			#change from pyhop plan to MIDCA plan
			midcaPlan = plans.Plan([plans.Action(self.operators[action[0]], *list(action[1:])) for action in pyhopPlan], goals)
			
			if verbose >= 1:
				print "Planning complete."
			if verbose >= 2:
				print "Plan: ", midcaPlan
			#save new plan
			if midcaPlan != None:
				self.mem.get(self.mem.GOAL_GRAPH).addPlan(midcaPlan)

	def pyhop_state_from_world(self, world, name = "state"):
		s = pyhop.State(name)
		s.pos = {}
		s.clear = {}
		s.holding = False
		s.fire = {}
		s.free = {}
		s.fire_ext_avail = set()
		s.holdingfireext = None
		blocks = []
		for objname in world.objects:
			if world.objects[objname].type.name == "BLOCK" and objname != "table":
				blocks.append(objname)
			elif world.objects[objname].type.name == "ARSONIST":
				s.free[objname] = False
		for atom in world.atoms:
			if atom.predicate.name == "clear":
				s.clear[atom.args[0].name] = True
			elif atom.predicate.name == "holding":
				s.holding = atom.args[0].name
			elif atom.predicate.name == "fire-extinguisher":
				s.fire_ext_avail.add(atom.args[0].name)
			elif atom.predicate.name == "holdingextinguisher":
				s.holdingfireext = atom.args[0].name
			elif atom.predicate.name == "arm-empty":
				s.holding = False
			elif atom.predicate.name == "on":
				s.pos[atom.args[0].name] = atom.args[1].name
			elif atom.predicate.name == "on-table":
				s.pos[atom.args[0].name] = "table"
			elif atom.predicate.name == "onfire":
				s.fire[atom.args[0].name] = True
			elif atom.predicate.name == "free":
				s.free[atom.args[0].name] = True
		for block in blocks:
			if block not in s.clear:
				s.clear[block] = False
			if block not in s.fire:
				s.fire[block] = False
			if block not in s.pos:
				s.pos[block] = "in-arm"
		return s
	
	#note: str(arg) must evaluate to the name of the arg in the world representation for this method to work.
	def pyhop_tasks_from_goals(self, goals):
		alltasks = []
		blkgoals = pyhop.Goal("goals")
		blkgoals.pos = {}
		for goal in goals:
			#extract predicate
			if 'predicate' in goal.kwargs:
				predicate = str(goal.kwargs['predicate'])
			elif 'Predicate' in goal.kwargs:
				predicate = str(goal.kwargs['Predicate'])
			elif goal.args:
				predicate = str(goal.args[0])
			else:
				raise ValueError("Goal " + str(goal) + " does not translate to a valid pyhop task")
			args = [str(arg) for arg in goal.args]
			if args[0] == predicate:
				args.pop(0)
			if predicate == "on":
				blkgoals.pos[args[0]] = args[1]
			elif predicate == 'on-table':
				blkgoals.pos[args[0]] = 'table'
			elif predicate == "onfire" and 'negate' in goal and goal['negate'] == True:
				alltasks.append(("put_out", args[0]))
			elif predicate == "free" and 'negate' in goal and goal['negate'] == True:
				alltasks.append(("catch_arsonist", args[0]))
			else:
				raise Exception("No task corresponds to predicate " + predicate)
		if blkgoals.pos:
			alltasks.append(("move_blocks", blkgoals))
		return alltasks	
	
	
