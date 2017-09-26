
from goalgen import *

class NewGuide:
	
	def __init__(self, useTFBase, useTFFire, catchArsonist, useMA, prioritizeNew):
		self.tfBase = useTFBase
		self.tfFire = useTFFire
		self.catchArsonist = catchArsonist
		self.useMA = useMA
		self.prioritizeNew = prioritizeNew
		self.step = 0
	
	def init(self, world, mem, memKeys):
		self.memKeys = memKeys	
		if self.tfBase:
			self.tfGen = gengoal.TFStackGen(mem, memKeys)
		else:
			self.predefGen = gengoal.ExogenousGoalGen(world)
		self.fireGen = gengoal.TFFireGen(mem, memKeys)
		if self.useMA:
			self.arsonGen = gengoal.XPGoalGen(mem, memKeys)
		else:
			self.arsonGen = gengoal.ArsonistCatcher(mem, memKeys)
		self.mem = mem
		mem.set(self.memKeys.MEM_GOALS, goalorg.GoalQueue())		
	
	def prioritize(self, goals):
		for goal in goals:
			goal.priority += float(self.step) / (self.step + 1)
	
	def get_goals(self, world, verbose, goalgen):
		goals = goalgen.gen_goals(verbose)
		if not goals or [None] == goals:
			goals = []
		if not hasattr(goals, '__iter__'):
			goals = [goals]
		if self.prioritizeNew:
			self.prioritize(goals)
		return goals
	
	def get_new_goals(self, world, verbose = 2):
		if self.tfBase:
			newgoals = self.get_goals(world, verbose, self.tfGen)
			if verbose >= 2:
				print "TF tree stacking goal generator activated. Goals:"
				for goal in newgoals:
					print "\t", goal, "  ",
				print
			if newgoals:
				self.mem._update(self.memKeys.MEM_GOALS, newgoals)
		else:
			newgoals = self.get_goals(world, verbose, self.predefGen)
			if verbose >= 2:
				print "Loading from predefined goals. Goals:"
				for goal in newgoals:
					print "\t", goal, "  ",
				print
			if newgoals:
				self.mem._update(self.memKeys.MEM_GOALS, newgoals)
		if self.tfFire:
			newgoals = self.get_goals(world, verbose, self.fireGen)
			if verbose >= 2:
				print "TF tree fire goal generator activated. Goals:"
				for goal in newgoals:
					print "\t", goal, "  ",
				print
			if newgoals:
				self.mem._update(self.memKeys.MEM_GOALS, newgoals)
		if self.catchArsonist:
			newgoals = self.get_goals(world, verbose, self.arsonGen)
			if self.prioritizeNew:
				self.prioritize(newgoals)
			if verbose >= 2:
				if not self.useMA:
					print "Simulated ",
				print "GDA K-track goal generation activated. Goals:"
				for goal in newgoals:
					print "\t", goal, "  ",
				print
			if newgoals:
				self.mem._update(self.memKeys.MEM_GOALS, newgoals)
		self.step += 1
	
	def run(self, verbose = 2):
		world = self.mem.get(self.memKeys.MEM_STATES)[-1]
		self.get_new_goals(world, verbose)
		
		