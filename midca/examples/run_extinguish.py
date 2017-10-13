#!/usr/bin/env python 
import midca
from midca.worldsim import domainread, stateread
from midca.modules import simulator, guide, perceive, note, evaluate, simulator, intend, planning, act
from midca import base
import inspect, os

# Domain Specific Imports
from midca.domains.blocksworld import util
from midca.domains.blocksworld.plan import methods, operators

DECLARE_METHODS_FUNC = methods.declare_methods
DECLARE_OPERATORS_FUNC = operators.declare_ops

thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

MIDCA_ROOT = thisDir + "/../"

domainFile = MIDCA_ROOT + "domains/blocksworld/domains/arsonist_extinguish.sim"
stateFile = MIDCA_ROOT + "domains/blocksworld/states/extinguisher_state.sim"
extinguish = True

argsPyHopPlanner = [util.pyhop_state_from_world,
					util.pyhop_tasks_from_goals,
					DECLARE_METHODS_FUNC,
					DECLARE_OPERATORS_FUNC,
					extinguish]

world = domainread.load_domain(domainFile)
stateread.apply_state_file(world, stateFile)
#creates a PhaseManager object, which wraps a MIDCA object
myMidca = base.PhaseManager(world, display = util.asqiiDisplay, verbose=4)
#add phases by name
for phase in ["Simulate", "Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
	myMidca.append_phase(phase)

#add the modules which instantiate basic blocksworld operation
myMidca.append_module("Simulate", simulator.MidcaActionSimulator())
myMidca.append_module("Simulate", simulator.ASCIIWorldViewer())
myMidca.append_module("Perceive", perceive.PerfectObserver())
myMidca.append_module("Interpret", note.ADistanceAnomalyNoter())
#myMidca.append_module("Interpret", guide.UserGoalInput())
myMidca.append_module("Eval", evaluate.SimpleEval())
myMidca.append_module("Intend", intend.SimpleIntend())
myMidca.append_module("Plan", planning.PyHopPlanner(*argsPyHopPlanner))
myMidca.append_module("Act", act.SimpleAct())

myMidca.insert_module('Simulate', simulator.ArsonSimulator(arsonChance = 0.3, arsonStart = 5), 1)
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
#myMidca.initGoalGraph(cmpFunc = preferFire)
myMidca.init()
myMidca.run()

'''
The code below would print out MIDCA's goal set for the first 20 phases of the run above. Note that any memory values can be accessed in this way, assuming that the storeHistory value was set to True during the run. This code is left as an example, but commented out because it will throw an error if fewer than 20 cycles were simulated.

for i in range(20):
	print myMidca.history[i].mem.get(myMidca.midca.mem.CURRENT_GOALS)

'''
