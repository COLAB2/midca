#!/usr/bin/env python
from MIDCA.worldsim import domainread, stateread
from MIDCA.modules import simulator, perceive, note, guide, evaluate, intend, planning, act
from MIDCA import base

import inspect, os

# domain specific imports
from MIDCA.domains.blocksworld import util
from MIDCA.domains.blocksworld.plan import methods_mortar, operators_mortar


'''
Simulation of tower construction and arson prevention in blocksworld. Uses
TF-trees and simulated Meta-AQUA connection to autonomously generate goals.
'''


thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

MIDCA_ROOT = thisDir + "/../"

### Domain Specific Variables
DOMAIN_ROOT = MIDCA_ROOT + "domains/blocksworld/"
DOMAIN_FILE = DOMAIN_ROOT + "domains/arsonist_mortar.sim"
STATE_FILE = DOMAIN_ROOT + "states/defstate_mortar.sim"
DISPLAY_FUNC = util.asqiiDisplay
DECLARE_METHODS_FUNC = methods_mortar.declare_methods
DECLARE_OPERATORS_FUNC = operators_mortar.declare_ops
GOAL_GRAPH_CMP_FUNC = util.preferApprehend

MORTAR_COUNT = 5
extinguish=False
mortar=True

world = domainread.load_domain(DOMAIN_FILE)
# for state file, need to add number of mortar blocks to begin with
state_str = open(STATE_FILE).read() # first read file
# now add new mortar blocks
for i in range(MORTAR_COUNT):
    state_str+="MORTARBLOCK(M"+str(i)+")\n"
    state_str+="available(M"+str(i)+")\n"
# now load the state    
stateread.apply_state_str(world, state_str)

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
myMidca.append_module("Interpret", guide.UserGoalInput())
myMidca.append_module("Eval", evaluate.SimpleEval())
myMidca.append_module("Intend", intend.SimpleIntend())
myMidca.append_module("Plan", planning.PyHopPlanner(util.mortar_pyhop_state_from_world,
                                                    util.mortar_pyhop_tasks_from_goals,
                                                    DECLARE_METHODS_FUNC,
                                                    DECLARE_OPERATORS_FUNC,
                                                    extinguish,
                                                    mortar))
myMidca.append_module("Act", act.SimpleAct())
#myMidca.insert_module('Simulate', simulator.ArsonSimulator(arsonChance = 0.0, arsonStart = 10), 1)
#myMidca.insert_module('Simulate', simulator.FireReset(), 0)
myMidca.insert_module('Interpret', guide.TFStack(), 1)
#myMidca.insert_module('Interpret', guide.TFFire(), 2)
myMidca.insert_module('Interpret', guide.ReactiveApprehend(), 3)
myMidca.insert_module('Eval', evaluate.MortarScorer(), 1) # this needs to be a 1 so that Scorer happens AFTER SimpleEval


#tells the PhaseManager to copy and store MIDCA states so they can be accessed later.
myMidca.storeHistory = True
myMidca.initGoalGraph()
myMidca.init()
myMidca.run()

'''
The code below would print out MIDCA's goal set for the first 20 phases of the run above. Note that any memory values can be accessed in this way, assuming that the storeHistory value was set to True during the run. This code is left as an example, but commented out because it will throw an error if fewer than 20 cycles were simulated.

for i in range(20):
    print myMidca.history[i].mem.get(myMidca.midca.mem.CURRENT_GOALS)

'''
