#!/usr/bin/env python 
import midca
from midca.examples import predicateworld
from midca.modules import simulator, guide, planning
import inspect, os

thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

MIDCA_ROOT = thisDir + "/../"

myMidca = predicateworld.UserGoalsMidca(domainFile = MIDCA_ROOT + "worldsim/domains/arsonist.sim", stateFile = MIDCA_ROOT + "worldsim/states/extinguisher_state.sim")

myMidca.clear_phase("Plan")

myMidca.insert_module("Plan", planning.PyHopPlanner2(), 0)
myMidca.insert_module('Simulate', simulator.ArsonSimulator(arsonChance = 0.3, arsonStart = 2), 1)
myMidca.insert_module('Interpret', guide.TFStack(), 1)
myMidca.insert_module('Interpret', guide.TFFire(), 2)

def preferFire(goal1, goal2):
	if 'predicate' not in goal1 or 'predicate' not in goal2:
		return 0
	elif goal1['predicate'] == 'onfire' and goal2['predicate'] != 'onfire':
		return -1
	elif goal1['predicate'] != 'onfire' and goal2['predicate'] == 'onfire':
		return 1
	return 0

#tells the PhaseManager to copy and store MIDCA states so they can be accessed later.
myMidca.storeHistory = True
myMidca.initGoalGraph(cmpFunc = preferFire)
myMidca.init()
myMidca.run()

'''
The code below would print out MIDCA's goal set for the first 20 phases of the run above. Note that any memory values can be accessed in this way, assuming that the storeHistory value was set to True during the run. This code is left as an example, but commented out because it will throw an error if fewer than 20 cycles were simulated.

for i in range(20):
	print myMidca.history[i].mem.get(myMidca.midca.mem.CURRENT_GOALS)

'''
