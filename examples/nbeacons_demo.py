#!/usr/bin/env python
import MIDCA
from MIDCA import base
from MIDCA.examples import predicateworld
from MIDCA.modules import simulator, guide, evaluate, perceive
from MIDCA.worldsim import domainread, stateread, worldsim
import inspect, os

'''
Simulation of the NBEACONS domain (adapted from marsworld in [Dannenhauer and Munoz-Avila 2015]).
'''

def pwrapper(x):
    print(str(x))

# Setup
thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
MIDCA_ROOT = thisDir + "/../"

# Domain & State Files  
domainFile = MIDCA_ROOT + "worldsim/domains/sample_domain.sim"
stateFile = MIDCA_ROOT + "worldsim/states/sample_state.sim"

# Load domain
world = domainread.load_domain(domainFile)

# Load state

#   for state file, need to add number of mortar blocks to begin with
state_str = open(stateFile).read() # first read file
# now add new mortar blocks
for i in range(self.currMortarCount):
    state_str+="MORTARBLOCK(M"+str(i)+")\n"
    state_str+="available(M"+str(i)+")\n"
# now load the state    
stateread.apply_state_str(self.world, state_str)




# Creates a PhaseManager object, which wraps a MIDCA object
myMidca = base.PhaseManager(world, display=pwrapper, verbose=4)

# Add phases by name
for phase in ["Simulate", "Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
    myMidca.append_phase(phase)

# Add the modules which instantiate basic operation
myMidca.append_module("Simulate", simulator.MidcaActionSimulator())
myMidca.append_module("Perceive", perceive.PerfectObserver())
myMidca.append_module("Interpret", guide.UserGoalInput())
myMidca.append_module("Eval", evaluate.SimpleEval())
myMidca.append_module("Intend", intend.SimpleIntend())
myMidca.append_module("Plan", planning.GenericPyhopPlanner(
    sample_methods.declare_methods, sample_operators.declare_ops)) # set up planner for sample domain
myMidca.append_module("Act", act.SimpleAct())

# Set world viewer to output text
myMidca.set_display_function(pwrapper) 

# Tells the PhaseManager to copy and store MIDCA states so they can be accessed later.
# Note: Turning this on drastically increases MIDCA's running time.
myMidca.storeHistory = False
myMidca.mem.logEachAccess = False

# Initialize and start running!
myMidca.init()
myMidca.run()


thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
MIDCA_ROOT = thisDir + "/../"

myMidca = predicateworld.UserGoalsMidca(domainFile = MIDCA_ROOT + "worldsim/domains/arsonist.sim", stateFile = MIDCA_ROOT + "worldsim/states/defstate.sim")

myMidca.insert_module('Simulate', simulator.ArsonSimulator(arsonChance = 0.9, arsonStart = 10), 1)
myMidca.insert_module('Simulate', simulator.FireReset(), 0)
myMidca.insert_module('Interpret', guide.TFStack(), 1)
myMidca.insert_module('Interpret', guide.TFFire(), 2)
myMidca.insert_module('Interpret', guide.ReactiveApprehend(), 3)
myMidca.insert_module('Eval', evaluate.Scorer(), 1) # this needs to be a 1 so that Scorer happens AFTER SimpleEval

def preferApprehend(goal1, goal2):
    if 'predicate' not in goal1 or 'predicate' not in goal2:
        return 0
    elif goal1['predicate'] == 'free' and goal2['predicate'] != 'free':
        return -1
    elif goal1['predicate'] != 'free' and goal2['predicate'] == 'free':
        return 1
    elif goal1['predicate'] == 'onfire' and goal2['predicate'] != 'onfire':
        return -1
    elif goal1['predicate'] != 'onfire' and goal2['predicate'] == 'onfire':
        return 1
    return 0

#tells the PhaseManager to copy and store MIDCA states so they can be accessed later.
myMidca.storeHistory = True
myMidca.initGoalGraph(cmpFunc = preferApprehend)
myMidca.init()
myMidca.run()

'''
The code below would print out MIDCA's goal set for the first 20 phases of the run above. Note that any memory values can be accessed in this way, assuming that the storeHistory value was set to True during the run. This code is left as an example, but commented out because it will throw an error if fewer than 20 cycles were simulated.

for i in range(20):
    print myMidca.history[i].mem.get(myMidca.midca.mem.CURRENT_GOALS)

'''
