import copy
from midca.modules.interpret._adist import ADistance, ChangeFinder, WindowPair, Interval
from midca.domains.nbeacons import nbeacons_util
import sys

class StateDiscrepancyDetector:
	'''
	Uses Immediate Expectations to detect discrepancies.
	For now, this only looks at effects of actions.
	'''
	def init(self, world, mem):
		self.world = world
		self.mem = mem

	def run(self, cycle, verbose=2):
		self.verbose = verbose
		trace = self.mem.trace
		if trace:
			trace.add_module(cycle,self.__class__.__name__)

		last_actions = None
		try:
			last_actions = self.mem.get(self.mem.ACTIONS)[-1]
			if self.verbose >= 2: print(("last_actions are "+str(list(map(str,last_actions)))))
			# for now assume just one action
			if len(last_actions) != 1:
				if self.verbose >= 1: print(("Agent has "+str(len(last_actions))+" previous actions, will not proceed"))
				if trace:
					trace.add_data("DISCREPANCY", None)
					trace.add_data("EXPECTED", None)
					trace.add_data("ACTUAL", None)
				return
		except:
			if self.verbose >= 1:
				print("No actions executed, skipping State Discrepancy Detection")

			return
		last_action = last_actions[0]
		copy_world = self.mem.get(self.mem.STATES)[-2]
		try:
			#print " attempting to execute action "+str(last_action)+" on preivous state:"
			#nbeacons_util.drawNBeaconsScene(copy_world)
			copy_world.apply_midca_action(last_action)

		except:
			print("Exception trying action "+str(last_action)+" is "+str(sys.exc_info()[0]))
			print("Previous action "+str(last_action)+" not applicable, agent did not execute an action during last act phase")
		world_diff = self.world.diff(copy_world)

		# we don't care about activated discrepancies right now
		# beacon failures are only to ensure there is always a goal for the agent
		# remove any 'activated' beacons
		expected = world_diff[0]
		actual = world_diff[1]
		expected_no_activate = []
		i = 0
		for exp_atom in expected:
			if 'activated' in str(exp_atom):
				if self.verbose >= 1: print('  ignoring activated atoms in discrepancies')
			else:
				expected_no_activate.append(exp_atom)
			i+=1

		expected = expected_no_activate

		#print("World diff returned : "+str(world_diff))
		if len(expected) > 0 or len(actual) > 0:
			if self.verbose >= 1: print(("Expected "+str(list(map(str,expected)))+ " but got "+str(list(map(str,actual)))))
		else:
			if self.verbose >= 1: print("No Discrepancy Detected")
		is_discrepancy = not (len(expected) == 0 and len(actual) == 0)

		if is_discrepancy:
			self.mem.set(self.mem.DISCREPANCY,(expected,actual))
		else:
			self.mem.set(self.mem.DISCREPANCY,None)

		if trace:
			trace.add_data("DISCREPANCY", is_discrepancy)
			trace.add_data("EXPECTED", str(list(map(str,expected))))
			trace.add_data("ACTUAL", str(list(map(str,actual))))

		return
