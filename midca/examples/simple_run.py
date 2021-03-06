#!/usr/bin/env python 
import midca
from midca.examples import predicateworld
from midca.logging import Logger
import inspect, os

# Domain Specific Imports
from midca.domains.blocksworld import util
from midca.domains.blocksworld.plan import methods, operators

'''
This script runs a simple version of MIDCA in blocksworld in which fires do not start, all goals are input by the user, and state changes are only caused by MIDCA actions and user intervention through a text interface.
'''

thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

MIDCA_ROOT = thisDir + "/../"

DECLARE_METHODS_FUNC = methods.declare_methods
DECLARE_OPERATORS_FUNC = operators.declare_ops
argsPyHopPlanner = [util.pyhop_state_from_world,
					util.pyhop_tasks_from_goals,
					DECLARE_METHODS_FUNC,
					DECLARE_OPERATORS_FUNC]


myMidca = predicateworld.UserGoalsMidca(domainFile = MIDCA_ROOT + 
									"domains/blocksworld/arsonist.sim", 
									stateFile = MIDCA_ROOT + "domains/blocksworld/states/defstate_fire.sim", 
									argsPyHopPlanner=argsPyHopPlanner)
#tells the PhaseManager to copy and store MIDCA states so they can be accessed later. Note: this drastically increases MIDCA's running time.
myMidca.storeHistory = True
myMidca.init()
myMidca.run()

'''
#The code below would print out MIDCA's goal set for the first 20 phases of the run above. Note that any memory values can be accessed in this way, assuming that the storeHistory value was set to True during the run. This code is left as an example, but commented out because it will throw an error if fewer than 20 cycles were simulated.

for i in range(20):
	print myMidca.history[i].mem.get(myMidca.midca.mem.CURRENT_GOALS)
'''

