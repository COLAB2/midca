#!/usr/bin/env python
import midca
from midca.examples import predicateworld
from midca.worldsim import domainread, stateread
from midca.modules import simulator, perceive, note, guide, evaluate, intend, planning, act
from midca.metamodules import monitor, control, interpret, metaintend,  plan
from midca.modules.gens import goaltransform
from midca import base

# Domain Specific Imports
from midca.domains.restaurant_domain import util
from midca.domains.restaurant_domain.plan import restaurant_methods, restaurant_operators

import inspect, os,copy

'''
Simulation of tower construction and arson prevention in blocksworld. Uses
TF-trees and simulated Meta-AQUA connection to autonomously generate goals.
'''

Money = 50

thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

MIDCA_ROOT = thisDir + "/../"

DOMAIN_ROOT = MIDCA_ROOT + "domains/restaurant_domain/"
DOMAIN_FILE = DOMAIN_ROOT + "domains/restaurant.sim"
STATE_FILE = DOMAIN_ROOT + "states/restaurant_state.sim"

DISPLAY_FUNC = util.shopping_display
DECLARE_METHODS_FUNC = restaurant_methods.declare_methods
DECLARE_OPERATORS_FUNC = restaurant_operators.declare_ops
GOAL_GRAPH_CMP_FUNC = util.preferApprehend

world = domainread.load_domain(DOMAIN_FILE)

# for state file, need to add number of mortar blocks to begin with
state_str = open(STATE_FILE).read() # first read file

# now load the state    
stateread.apply_state_str(world, state_str)
stateread.apply_state_file(world, STATE_FILE)

print(world)
#creates a PhaseManager object, which wraps a MIDCA object
myMidca = base.PhaseManager(world, display = DISPLAY_FUNC, verbose=4, metaEnabled=True)

initial_world = copy.deepcopy(world)
#add phases by name
for phase in ["Simulate", "Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
    myMidca.append_phase(phase)

myMidca.append_module("Simulate", simulator.MidcaActionSimulator())
myMidca.append_module("Perceive", perceive.PerfectObserver())
myMidca.append_module("Interpret", note.ADistanceAnomalyNoter())
myMidca.append_module("Interpret", guide.SimpleMortarGoalGen_Restaurant(STATE_FILE,state_str,Money))
myMidca.append_module("Eval", evaluate.SimpleEval_Restaurant())
myMidca.append_module("Intend", intend.SimpleIntend_Restaurant())
myMidca.append_module("Plan", planning.PyHopPlanner_temporary(util.pyhop_state_from_world_restaurant,
                                                    util.pyhop_tasks_from_goals_restaurant,
                                                    DECLARE_METHODS_FUNC,
                                                    DECLARE_OPERATORS_FUNC))
myMidca.append_module("Act", act.SimpleAct_temporary())

#tells the PhaseManager to copy and store MIDCA states so they can be accessed later.
myMidca.storeHistory = True
myMidca.initGoalGraph()
myMidca.init()
myMidca.run()

'''
for phase in ["Simulate", "Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
    myMidca.append_phase(phase)

    #add the modules which instantiate basic blocksworld operation
myMidca.append_module("Simulate", simulator.MidcaActionSimulator())
myMidca.append_module("Simulate", simulator.ASCIIWorldViewer())
myMidca.append_module("Perceive", perceive.PerfectObserver())
myMidca.append_module("Interpret", note.ADistanceAnomalyNoter())
#myMidca.append_module("Interpret", guide.UserGoalInput())
myMidca.append_module("Eval", evaluate.SimpleEval_construction())
myMidca.append_module("Intend", intend.SimpleIntend_construction())
myMidca.append_module("Plan", planning.PyHopPlanner_construction(extinguish,mortar))
myMidca.append_module("Act", act.SimpleAct())
#myMidca.insert_module('Simulate', simulator.ArsonSimulator(arsonChance = 0.0, arsonStart = 10), 1)
#myMidca.insert_module('Simulate', simulator.FireReset(), 0)
myMidca.insert_module('Interpret', guide.SimpleMortarGoalGen_construction(stateFile,state_str,T), 1)
#myMidca.insert_module('Interpret', guide.TFFire(), 2)
myMidca.insert_module('Interpret', guide.ReactiveApprehend(), 3)
# add meta layer phases
#for phase in ["Monitor", "Interpret", "Eval", "Intend", "Plan", "Control"]:
for phase in ["Monitor", "Interpret", "Intend", "Plan", "Control"]:
    myMidca.append_meta_phase(phase)

# add meta layer modules
myMidca.append_meta_module("Monitor", monitor.MRSimpleMonitor())
myMidca.append_meta_module("Interpret", interpret.MRSimpleDetect_construction())
myMidca.append_meta_module("Interpret", interpret.MRSimpleGoalGenForGoalTrans())
myMidca.append_meta_module("Intend", metaintend.MRSimpleIntend())
myMidca.append_meta_module("Plan", plan.MRSimplePlanner())
myMidca.append_meta_module("Control", control.MRSimpleControl1())



#tells the PhaseManager to copy and store MIDCA states so they can be accessed later.
myMidca.storeHistory = True
myMidca.initGoalGraph()
myMidca.init()
#a = goaltransform.choose(myMidca.midca.world , "stable-on(A_,B_)")
#tree = cl.Tree()
#objecttree = cl.ObjectTree()
#cl.implement(domainFile,tree,objecttree)
#tree.printtree()
#objecttree.printtree()
myMidca.run()


The code below would print out MIDCA's goal set for the first 20 phases of the run above. Note that any memory values can be accessed in this way, assuming that the storeHistory value was set to True during the run. This code is left as an example, but commented out because it will throw an error if fewer than 20 cycles were simulated.

for i in range(20):
    print myMidca.history[i].mem.get(myMidca.midca.mem.CURRENT_GOALS)

'''
