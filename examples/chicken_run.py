from __future__ import print_function
#!/usr/bin/env python 
import MIDCA
from MIDCA.examples import predicateworld
from MIDCA.logging import Logger
import inspect, os

from MIDCA.modules import planning
from MIDCA.modules._plan import sample_methods, sample_operators

from MIDCA import base
from MIDCA.worldsim import domainread, stateread, worldsim, blockstate, scene
from MIDCA.modules import simulator, perceive, note, guide, evaluate, intend, planning, act

'''
This script runs a simple version of MIDCA in blocksworld in which fires do not start, all goals are input by the user, and state changes are only caused by MIDCA actions and user intervention through a text interface.
'''

thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

MIDCA_ROOT = thisDir + "/../"

domainFile = MIDCA_ROOT + "worldsim/domains/sample_domain.sim"
stateFile = MIDCA_ROOT + "worldsim/states/sample_state.sim"

world = domainread.load_domain(domainFile)
stateread.apply_state_file(world, stateFile)
    #creates a PhaseManager object, which wraps a MIDCA object
myMidca = base.PhaseManager(world, display = print, verbose=4)
    #add phases by name
for phase in ["Simulate", "Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
    myMidca.append_phase(phase)

    #add the modules which instantiate basic blocksworld operation
myMidca.append_module("Simulate", simulator.MidcaActionSimulator())
myMidca.append_module("Perceive", perceive.PerfectObserver())
myMidca.append_module("Interpret", note.ADistanceAnomalyNoter())
#myMidca.append_module("Interpret", guide.UserGoalInput())
myMidca.append_module("Eval", evaluate.SimpleEval())
myMidca.append_module("Intend", intend.SimpleIntend())
myMidca.append_module("Act", act.SimpleAct())

#set up planner for chicken domain
myMidca.append_module("Plan", planning.GenericPyhopPlanner(
    sample_methods.declare_methods, sample_operators.declare_ops))
myMidca.set_display_function(print) #set world viewer to output text

#tells the PhaseManager to copy and store MIDCA states so they can be accessed later. Note: this drastically increases MIDCA's running time.
myMidca.storeHistory = False
myMidca.mem.logEachAccess = False

myMidca.init()
myMidca.run()

'''
#The code below would print out MIDCA's goal set for the first 20 phases of the run above. Note that any memory values can be accessed in this way, assuming that the storeHistory value was set to True during the run. This code is left as an example, but commented out because it will throw an error if fewer than 20 cycles were simulated.

for i in range(20):
	print myMidca.history[i].mem.get(myMidca.midca.mem.CURRENT_GOALS)
'''
