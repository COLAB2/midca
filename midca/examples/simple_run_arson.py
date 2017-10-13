#!/usr/bin/env python
from midca import base
from midca.modules import simulator, guide, perceive, note, evaluate, intend, planning, act
from midca.worldsim import stateread, domainread
import inspect, os

# Domain Specific Imports
from midca.domains.blocksworld import util
from midca.domains.blocksworld.plan import methods_extinguish, operators_extinguish

thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

MIDCA_ROOT = thisDir + "/../"

### Domain Specific Variables
DOMAIN_ROOT = MIDCA_ROOT + "domains/blocksworld/"
DOMAIN_FILE = DOMAIN_ROOT + "domains/arsonist_extinguish.sim"
STATE_FILE = DOMAIN_ROOT + "states/extinguisher_state.sim"
DISPLAY_FUNC = util.asqiiDisplay
DECLARE_METHODS_FUNC = methods_extinguish.declare_methods
DECLARE_OPERATORS_FUNC = operators_extinguish.declare_ops
GOAL_GRAPH_CMP_FUNC = util.preferFire


world = domainread.load_domain(DOMAIN_FILE)
stateread.apply_state_file(world, STATE_FILE)
    #creates a PhaseManager object, which wraps a MIDCA object
myMidca = base.PhaseManager(world, display = DISPLAY_FUNC, verbose=4)
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
myMidca.append_module("Plan", planning.PyHopPlanner(util.pyhop_state_from_world,
                                                    util.pyhop_tasks_from_goals,
                                                    DECLARE_METHODS_FUNC,
                                                    DECLARE_OPERATORS_FUNC))

myMidca.append_module("Act", act.SimpleAct())

myMidca.insert_module('Simulate', simulator.ArsonSimulator(arsonChance = 0.3, arsonStart = 2), 1)
myMidca.insert_module('Interpret', guide.TFStack(), 1)
myMidca.insert_module('Interpret', guide.TFFire(), 2)



#tells the PhaseManager to copy and store MIDCA states so they can be accessed later. Note: this drastically increases MIDCA's running time.
myMidca.storeHistory = True
myMidca.initGoalGraph(cmpFunc = GOAL_GRAPH_CMP_FUNC)
myMidca.init()
myMidca.run()

'''
The code below would print out MIDCA's goal set for the first 20 phases of the run above. Note that any memory values can be accessed in this way, assuming that the storeHistory value was set to True during the run. This code is left as an example, but commented out because it will throw an error if fewer than 20 cycles were simulated.

for i in range(20):
    print myMidca.history[i].mem.get(myMidca.midca.mem.CURRENT_GOALS)

'''
