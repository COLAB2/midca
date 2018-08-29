#!/usr/bin/env python
from midca import base
from midca.modules import simulator, guide, evaluate, perceive, note, intend, planning, act
from midca.worldsim import stateread
from midca.worldsim import pddl_num_read as pddlread
import inspect, os

# Domain Specific Imports
from midca.domains.ffdomain.minecraft import minecraft_util
# from midca.domains.minecraft import PDDL_util

### this script is not working for now. 



'''
Simulation of tower construction and arson prevention in blocksworld. Uses
TF-trees and simulated Meta-AQUA connection to autonomously generate goals.
'''

thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
GOAL_GRAPH_CMP_FUNC = minecraft_util.preferSurvive

MIDCA_ROOT = thisDir + "/../"


### Domain Specific Variables
# DOMAIN_ROOT = MIDCA_ROOT + "domains/minecraft/"
# DOMAIN_FILE = DOMAIN_ROOT + "domains/domain.sim"
# STATE_FILE = DOMAIN_ROOT + "states/defstate.sim"

### Domain Specific Variables for JSHOP planner
DOMAIN_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/domain.pddl"
STATE_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/wood.1.pddl"

world = pddlread.load_domain(DOMAIN_FILE, STATE_FILE)
# stateread._apply_state_pddl(world, DOMAIN_FILE, STATE_FILE)
# creates a PhaseManager object, which wraps a MIDCA object
myMidca = base.PhaseManager(world, display='', verbose=4)
# add phases by name
for phase in ["Simulate", "Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
    myMidca.append_phase(phase)

# add the modules which instantiate basic blocksworld operation
myMidca.append_module("Simulate", simulator.MidcaActionSimulator())
myMidca.append_module("Simulate", simulator.MidcaEventSimulator())
# myMidca.append_module("Simulate", simulator.ASCIIWorldViewer(display=DISPLAY_FUNC))
myMidca.append_module("Perceive", perceive.PerfectObserver())
myMidca.append_module("Interpret", guide.UserGoalInput())
myMidca.append_module("Eval", evaluate.SimpleEvalSubgoals())
myMidca.append_module("Intend", intend.SimpleIntendwithSubgoals())
myMidca.append_module("Plan", planning.MetricFFPlanner(
    minecraft_util.ff_goals_from_midca_goals,
    minecraft_util.ff_state_from_midca_world,
    DOMAIN_FILE,
    STATE_FILE
))
myMidca.append_module("Act", act.SimpleAct())
# myMidca.insert_module('Simulate', simulator.ArrowSimulator(arrowStart=1), 1)
# myMidca.insert_module('Simulate', simulator.AttackSimulator(skeletonStart=2), 1)
# myMidca.insert_module('Interpret', guide.TFFire(), 2)
myMidca.insert_module('Interpret', guide.ReactiveSurvive(), 3)
# tells the PhaseManager to copy and store MIDCA states so they can be accessed later.
myMidca.storeHistory = True
myMidca.initGoalGraph(cmpFunc=GOAL_GRAPH_CMP_FUNC)
myMidca.init()
myMidca.run()

'''
The code below would print out MIDCA's goal set for the first 20 phases of the run above. Note that any memory values can be accessed in this way, assuming that the storeHistory value was set to True during the run. This code is left as an example, but commented out because it will throw an error if fewer than 20 cycles were simulated.

for i in range(20):
    print myMidca.history[i].mem.get(myMidca.midca.mem.CURRENT_GOALS)

'''
#when the state changes I need to change the domain file for the planner :O