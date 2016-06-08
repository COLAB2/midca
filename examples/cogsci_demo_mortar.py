#!/usr/bin/env python
import MIDCA
from MIDCA.examples import predicateworld
from MIDCA.worldsim import domainread, stateread, worldsim, blockstate, scene
from MIDCA.modules import simulator, perceive, note, guide, evaluate, intend, planning, act
from MIDCA.metamodules import monitor, control, interpret, metaintend,  plan
from MIDCA import base

import inspect, os

'''
Simulation of tower construction and arson prevention in blocksworld. Uses
TF-trees and simulated Meta-AQUA connection to autonomously generate goals.
'''

MORTAR_COUNT = 4

thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

MIDCA_ROOT = thisDir + "/../"

domainFile = MIDCA_ROOT + "worldsim/domains/arsonist_mortar.sim"
stateFile = MIDCA_ROOT + "worldsim/states/defstate_mortar.sim"
extinguish=False
mortar=True
world = domainread.load_domain(domainFile)

# for state file, need to add number of mortar blocks to begin with
state_str = open(stateFile).read() # first read file
# now add new mortar blocks
for i in range(MORTAR_COUNT):
    state_str+="MORTARBLOCK(M"+str(i)+")\n"
    state_str+="available(M"+str(i)+")\n"
# now load the state    
stateread.apply_state_str(world, state_str)

stateread.apply_state_file(world, stateFile)
    #creates a PhaseManager object, which wraps a MIDCA object
myMidca = base.PhaseManager(world, display = predicateworld.asqiiDisplay, verbose=4, metaEnabled=True)

predicateworld.asqiiDisplay(world)
    #add phases by name
for phase in ["Simulate", "Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
    myMidca.append_phase(phase)

    #add the modules which instantiate basic blocksworld operation
myMidca.append_module("Simulate", simulator.MidcaActionSimulator())
myMidca.append_module("Simulate", simulator.ASCIIWorldViewer())
myMidca.append_module("Perceive", perceive.PerfectObserver())
myMidca.append_module("Interpret", note.ADistanceAnomalyNoter())
#myMidca.append_module("Interpret", guide.UserGoalInput())
myMidca.append_module("Eval", evaluate.SimpleEval())
myMidca.append_module("Intend", intend.SimpleIntend())
myMidca.append_module("Plan", planning.PyHopPlanner(extinguish,mortar))
myMidca.append_module("Act", act.SimpleAct())
#myMidca.insert_module('Simulate', simulator.ArsonSimulator(arsonChance = 0.0, arsonStart = 10), 1)
#myMidca.insert_module('Simulate', simulator.FireReset(), 0)
myMidca.insert_module('Interpret', guide.SimpleMortarGoalGen(), 1)
#myMidca.insert_module('Interpret', guide.TFFire(), 2)
myMidca.insert_module('Interpret', guide.ReactiveApprehend(), 3)
myMidca.insert_module('Eval', evaluate.MortarScorer(), 1) # this needs to be a 1 so that Scorer happens AFTER SimpleEval

# add meta layer phases
#for phase in ["Monitor", "Interpret", "Eval", "Intend", "Plan", "Control"]:
for phase in ["Monitor", "Interpret", "Intend", "Plan", "Control"]:
    myMidca.append_meta_phase(phase)

# add meta layer modules
myMidca.append_meta_module("Monitor", monitor.MRSimpleMonitor())
myMidca.append_meta_module("Interpret", interpret.MRSimpleDetect2())
myMidca.append_meta_module("Interpret", interpret.MRSimpleGoalGenForGoalTrans())
myMidca.append_meta_module("Intend", metaintend.MRSimpleIntend())
myMidca.append_meta_module("Plan", plan.MRSimplePlanner())
myMidca.append_meta_module("Control", control.MRSimpleControl())

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
