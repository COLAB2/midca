#!/usr/bin/env python 
import midca
from midca.examples import predicateworld
from midca.modules import simulator, guide, evaluate, perceive, assess 
import inspect, os

# Domain Specific Imports
from midca.domains.blocksworld import util
from midca.domains.blocksworld.plan import methods, operators

'''
Simulation of tower construction and arson prevention in blocksworld. Uses
TF-trees and Meta-AQUA connection to autonomously generate goals. Meta-AQUA
must be started and have opened sockets prior to launching MIDCA.
'''

thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

MIDCA_ROOT = thisDir + "/../"



writePort = 5150
readPort = 5151
DISPLAY_FUNC = util.asqiiDisplay
DECLARE_METHODS_FUNC = methods.declare_methods
DECLARE_OPERATORS_FUNC = operators.declare_ops
argsPyHopPlanner = [util.pyhop_state_from_world,
					util.pyhop_tasks_from_goals,
					DECLARE_METHODS_FUNC,
					DECLARE_OPERATORS_FUNC]
myMidca = predicateworld.UserGoalsMidca(domainFile = MIDCA_ROOT + "domains/blocksworld/domains/arsonist.sim", stateFile = MIDCA_ROOT + "domains/blocksworld/states/defstate.sim", display = DISPLAY_FUNC, argsPyHopPlanner=argsPyHopPlanner)

myMidca.append_module('Perceive', perceive.MAReporter(writePort))
myMidca.insert_module('Simulate', simulator.ArsonSimulator(arsonChance = 0.9, arsonStart = 2), 1)
myMidca.insert_module('Simulate', simulator.FireReset(), 0)
myMidca.insert_module('Interpret', guide.TFStack(), 1)
myMidca.insert_module('Interpret', guide.TFFire(), 2)
myMidca.insert_module('Interpret', assess.MAQuery(readPort), 3)
myMidca.insert_module('Eval', evaluate.Scorer(), 0)


def preferApprehend(goal1, goal2):
	if 'predicate' not in goal1 or 'predicate' not in goal2:
		return 0
	elif goal1['predicate'] == 'free' and goal2['predicate'] != 'free':
		return -1
	elif goal1['predicate'] != 'free' and goal2['predicate'] == 'free':
		return 1
	elif goal1['predicate'] == 'onfire' and goal2['predicate'] != 'onfire':
		return -1
	elif goal1['predicate'] != 'onfire' and goal2['predicate'] == 'onfire':
		return 1
	return 0

#tells the PhaseManager to copy and store MIDCA states so they can be accessed later.
myMidca.storeHistory = True
myMidca.initGoalGraph(cmpFunc = preferApprehend)
myMidca.init()
myMidca.run()

'''
The code below would print out MIDCA's goal set for the first 20 phases of the run above. Note that any memory values can be accessed in this way, assuming that the storeHistory value was set to True during the run. This code is left as an example, but commented out because it will throw an error if fewer than 20 cycles were simulated.

for i in range(20):
	print myMidca.history[i].mem.get(myMidca.midca.mem.CURRENT_GOALS)

'''
