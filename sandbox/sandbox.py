# File: sandbox.py
# Author: Dustin Dannenhauer
# 
# This file runs MIDCA in a sandbox like fashion, alllowing the user
# to specific resources and goals, and record data.

# 1. First I'm going to get a simple sandbox demo up and running with
# a few fire extinguishers and fires, where I also collect the data,
# and produce a csv file.

# 2. Then I'm going to implement the ability to read in a "tick" file,
# which will run certain events at certain times (i.e. remove a fire
# extinguisher at tick 10, or start a fire at tick 15)

# 3. After both of those are working, I'm going to go through and see
# about making everything as general as possible (so we could vary
# resources, etc)

import MIDCA

from MIDCA.examples import predicateworld
from MIDCA.modules import simulator, guide
import inspect, os
import xml.etree.ElementTree as ET

sampleTickFile = 'sandbox/sample_tick_events.csv'

thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

MIDCA_ROOT = thisDir + "/../"

##### [Initial Domain and State Files] #####
## Arsonist without extinguishers ##
arsonist_domain = "worldsim/domains/arsonist.sim"
arsonist_state = "worldsim/states/defstate_fire.sim"

## Arsonist with extinguisher domain ##
extinguisher_domain = "worldsim/domains/arsonist_extinguish.sim"
extinguisher_state = "worldsim/states/extinguisher_state.sim"

##### END [Initial Domain and State Files] #####

myMidca = predicateworld.UserGoalsMidca(domainFile = MIDCA_ROOT + extinguisher_domain, stateFile = MIDCA_ROOT + extinguisher_state, extinguish=True)

myMidca.insert_module('Simulate', simulator.ArsonSimulator(arsonChance = 0.3, arsonStart = 2, tickFile = sampleTickFile), 1)
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
myMidca.initGoalGraph(cmpFunc = preferFire)
myMidca.init()
myMidca.run()

'''
The code below would print out MIDCA's goal set for the first 20 phases of the run above. Note that any memory values can be accessed in this way, assuming that the storeHistory value was set to True during the run. This code is left as an example, but commented out because it will throw an error if fewer than 20 cycles were simulated.

for i in range(20):
	print myMidca.history[i].mem.get(myMidca.midca.mem.CURRENT_GOALS)

'''





