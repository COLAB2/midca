#!/usr/bin/env python 
import MIDCA
from MIDCA.examples import predicateworld
from MIDCA.logging import Logger
import inspect, os

thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

MIDCA_ROOT = thisDir + "/../"

myMidca = predicateworld.UserGoalsMidca(domainFile = MIDCA_ROOT + "worldsim/domains/arsonist.sim", stateFile = MIDCA_ROOT + "worldsim/states/defstate_fire.sim")

myMidca.logger.logOutput()
myMidca.mem.enableLogging(myMidca.logger)

#tells the PhaseManager to copy and store MIDCA states so they can be accessed later.
myMidca.storeHistory = True
myMidca.init()
myMidca.run()

'''
The code below would print out MIDCA's goal set for the first 20 phases of the run above. Note that any memory values can be accessed in this way, assuming that the storeHistory value was set to True during the run. This code is left as an example, but commented out because it will throw an error if fewer than 20 cycles were simulated.

for i in range(20):
	print myMidca.history[i].mem.get(myMidca.midca.mem.CURRENT_GOALS)

'''
