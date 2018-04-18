#!/usr/bin/env python
from midca import base
from midca.modules import simulator, guide, evaluate, perceive, note, intend, planning, act

from midca.worldsim import domainread, stateread
import inspect, os

# Domain Specific Imports
from midca.domains.logistics import logistics_util
from midca.domains.blocksworld.plan import methods, operators

### this script is not working for now. 

'''
Simulation of tower construction and arson prevention in blocksworld. Uses
TF-trees and simulated Meta-AQUA connection to autonomously generate goals.
'''

thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print thisDir

MIDCA_ROOT = thisDir + "/../"
print MIDCA_ROOT

### Domain Specific Variables
DOMAIN_ROOT = MIDCA_ROOT + "domains/logistics/"
DOMAIN_FILE = DOMAIN_ROOT + "domains/domain2.sim"
STATE_FILE = DOMAIN_ROOT + "states/defstate2.sim"

### Domain Specific Variables for JSHOP planner
JSHOP_DOMAIN_FILE = MIDCA_ROOT + "domains/jshop_domains/logistics/logistics"
JSHOP_STATE_FILE = MIDCA_ROOT + "domains/jshop_domains/logistics/problem"

world = domainread.load_domain(DOMAIN_FILE)
stateread.apply_state_file(world, STATE_FILE)
#creates a PhaseManager object, which wraps a MIDCA object
myMidca = base.PhaseManager(world, display = '', verbose=4)
#add phases by name
for phase in ["Simulate", "Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
    myMidca.append_phase(phase)

#add the modules which instantiate basic blocksworld operation
myMidca.append_module("Simulate", simulator.MidcaActionSimulator())
#myMidca.append_module("Simulate", simulator.ASCIIWorldViewer(display=DISPLAY_FUNC))
myMidca.append_module("Perceive", perceive.PerfectObserverWithThief())
myMidca.append_module("Interpret", guide.DeliverGoal())
#myMidca.append_module("Interpret", guide.UserGoalInput())
myMidca.append_module("Eval", evaluate.SimpleEval2())
myMidca.append_module("Intend", intend.WarehouseIntend())
myMidca.append_module("Plan", planning.JSHOP2Planner(logistics_util.jshop2_state_from_world,
                                                    logistics_util.jshop2_tasks_from_goals,
                                                    JSHOP_DOMAIN_FILE,
                                                    JSHOP_STATE_FILE
                                                    ))
myMidca.append_module("Act", act.SimpleAct())



#tells the PhaseManager to copy and store MIDCA states so they can be accessed later.
myMidca.storeHistory = True

myMidca.init()
myMidca.run()

'''
The code below would print out MIDCA's goal set for the first 20 phases of the run above. Note that any memory values can be accessed in this way, assuming that the storeHistory value was set to True during the run. This code is left as an example, but commented out because it will throw an error if fewer than 20 cycles were simulated.

for i in range(20):
    print myMidca.history[i].mem.get(myMidca.midca.mem.CURRENT_GOALS)

'''
