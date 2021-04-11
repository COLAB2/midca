import copy
from midca.modules.interpret._adist import ADistance, ChangeFinder, WindowPair, Interval
from midca.domains.nbeacons import nbeacons_util
import sys

class InformedDiscrepancyDetector:
	'''
	Performs Discrepancy Detection using Informed Expectations
	TODO: This is a stub for now, not implemented yet
	'''
	def __init__(self):
		pass

	def init(self, world, mem):
		self.world = world
		self.mem = mem

	def get_current_plan(self):
		'''
		Returns the current plan the agent is using
		'''
		pass

	def generate_inf_exp(self, plan, prev_action):
		'''
		Returns a set of atoms to check against the state given the previous action
		the agent executed and the current plan

		See Dannenhauer & Munoz-Avila IJCAI-2015 / Dannenhauer, Munoz-Avila, Cox IJCAI-2016
		for more information on informed expectations

		TODO: finish
		'''
		# sanity check, make sure prev_action is in the plan
		if prev_action not in plan:
			raise Exception("Cannot generate informed expectations: prev_action "+str(prev_action)+" was not given plan")

		exp = [] # expectations accumulator
		pass

	def run(self, cycle, verbose=2):
		prev_action = self.mem.get(self.mem.ACTIONS)[-1]
		curr_goals = self.mem.get(self.mem.CURRENT_GOALS)
		plan = self.mem.get(self.mem.GOAL_GRAPH).get_best_plan(curr_goals)
		inf_exp = self.generate_inf_exp(prev_action)
		for e in inf_exp:
			pass
