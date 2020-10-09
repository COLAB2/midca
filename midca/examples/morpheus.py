#!/usr/bin/env python
from midca import base
from midca.modules import simulator, guide, evaluate, perceive, note, intend, planning, act
from midca.modules._plan.asynch import asynch_communication
from midca.worldsim import domainread, stateread
import inspect, os

# Domain Specific Imports
from midca.domains.blocksworld import util
from midca.domains.blocksworld.plan import methods_multiAgent as methods, operators_multiAgent as operators


'''
Simulation of tower construction and arson prevention in blocksworld. Uses
TF-trees and simulated Meta-AQUA connection to autonomously generate goals.
'''

thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

MIDCA_ROOT = thisDir + "/../"

### Domain Specific Variables
DOMAIN_ROOT = MIDCA_ROOT + "domains/blocksworld/"
DOMAIN_FILE = DOMAIN_ROOT + "domains/arsonist_multiAgent.sim"
STATE_FILE = DOMAIN_ROOT + "states/defstate_multiAgent.sim"
DISPLAY_FUNC = util.asqiiDisplay
DECLARE_METHODS_FUNC = methods.declare_methods
DECLARE_OPERATORS_FUNC = operators.declare_ops
DECLARE_ACTIONS = asynch_communication
GOAL_GRAPH_CMP_FUNC = util.preferCommunication

world = domainread.load_domain(DOMAIN_FILE)
stateread.apply_state_file(world, STATE_FILE)
#creates a PhaseManager object, which wraps a MIDCA object
myMidca = base.PhaseManager(world, display = DISPLAY_FUNC, verbose=4)
#add phases by name
for phase in ["Simulate", "Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
    myMidca.append_phase(phase)

agent_name = "morpheus"
other_agent_name = "neo"

publish_simulator = "tcp://127.0.0.1:7000"
subscribe_simulator = "tcp://127.0.0.1:6000"


publish = "tcp://127.0.0.1:5000"
subscribe = "tcp://127.0.0.1:4000"

#add the modules which instantiate basic blocksworld operation
myMidca.append_module("Simulate", simulator.SendRemoteMidcaActionSimulator(publish_simulator, subscribe_simulator))
myMidca.append_module("Perceive", perceive.RecieveRemoteMidcaWorld(publish_simulator, subscribe_simulator))
myMidca.append_module("Perceive", perceive.PerfectObserver())
myMidca.append_module("Perceive", perceive.RecieveRequests(publish, subscribe, agent_name, other_agent_name))
myMidca.append_module("Perceive", simulator.ASCIIWorldViewer(display=DISPLAY_FUNC))
#myMidca.append_module("Perceive", perceive.UserGoalInput(agent_name))
myMidca.append_module("Interpret", guide.InterpretRequests(agent_name))
myMidca.append_module("Interpret", guide.GenerateRequests(agent_name, other_agent_name))
myMidca.append_module("Interpret", guide.EvaluateRequests(agent_name, other_agent_name))
myMidca.append_module("Eval", evaluate.SimpleEval())
myMidca.append_module("Intend", intend.SimpleIntendMultipleGoals())
myMidca.append_module("Plan", planning.PyHopPlanner(util.pyhop_state_from_world,
                                                    util.pyhop_tasks_from_goals,
                                                    DECLARE_METHODS_FUNC,
                                                    DECLARE_OPERATORS_FUNC))

myMidca.append_module("Act", act.SimpleAct(DECLARE_ACTIONS))
"""
myMidca.append_module("Interpret", note.ADistanceAnomalyNoter())
myMidca.append_module("Interpret", guide.UserGoalInput())
myMidca.append_module("Eval", evaluate.SimpleEval())
myMidca.append_module("Intend", intend.SimpleIntend())
myMidca.append_module("Plan", planning.PyHopPlanner(util.pyhop_state_from_world,
                                                    util.pyhop_tasks_from_goals,
                                                    DECLARE_METHODS_FUNC,
                                                    DECLARE_OPERATORS_FUNC))
myMidca.append_module("Act", act.SimpleAct())

myMidca.insert_module('Simulate', simulator.ArsonSimulator(arsonChance = 0.9, arsonStart = 10), 1)
myMidca.insert_module('Simulate', simulator.FireReset(), 0)
myMidca.insert_module('Interpret', guide.TFStack(), 1)
myMidca.insert_module('Interpret', guide.TFFire(), 2)
myMidca.insert_module('Interpret', guide.ReactiveApprehend(), 3)
myMidca.insert_module('Eval', evaluate.Scorer(), 1) # this needs to be a 1 so that Scorer happens AFTER SimpleEval
"""

#tells the PhaseManager to copy and store MIDCA states so they can be accessed later.
myMidca.storeHistory = True
myMidca.initGoalGraph(cmpFunc = GOAL_GRAPH_CMP_FUNC)
myMidca.init()
myMidca.run()

'''
The code below would print out MIDCA's goal set for the first 20 phases of the run above. Note that any memory values can be accessed in this way, assuming that the storeHistory value was set to True during the run. This code is left as an example, but commented out because it will throw an error if fewer than 20 cycles were simulated.

for i in range(20):
    print myMidca.history[i].mem.get(myMidca.midca.mem.CURRENT_GOALS)

'''
