from __future__ import print_function
#!/usr/bin/env python 
import MIDCA
from MIDCA.examples import predicateworld
from MIDCA.logging import Logger
import inspect, os

from MIDCA.modules import planning
from MIDCA.modules._plan import sample_methods, sample_operators

'''
This script runs a simple version of MIDCA in blocksworld in which fires do not start, all goals are input by the user, and state changes are only caused by MIDCA actions and user intervention through a text interface.
'''

thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

MIDCA_ROOT = thisDir + "/../"

myMidca = predicateworld.UserGoalsMidca(domainFile = MIDCA_ROOT + "worldsim/domains/sample_domain.sim", stateFile = MIDCA_ROOT + "worldsim/states/sample_state.sim")

#set up planner for chicken domain
myMidca.remove_module("Simulate", 1) #remove blocksworld viewer
myMidca.clear_phase("Plan")
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
