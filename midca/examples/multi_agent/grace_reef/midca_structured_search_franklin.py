#!/usr/bin/env python
import midca
from midca import base, ZmqInterface
from midca.modules import simulator, guide, evaluate, perceive, intend, planning, act, note, assess
from midca.worldsim import domainread, stateread
from midca.modules._plan.asynch import asynch_grace
import inspect, os, json, copy
import threading

# Domain Specific Imports
from midca.domains.grace import grace_util as nbeacons_util
# simulator
from midca.domains.grace.interface import tagworld

# Domain Specific Imports
from midca.domains.blocksworld import util, handlers
from midca.domains.blocksworld.plan import methods_multiAgent as methods, operators_multiAgent as operators


#initialize simulator
interface = tagworld.TagWorld(sub_ip = "tcp://127.0.0.1:7999",
                              sub_mine_ip = "tcp://127.0.0.1:6999" ,
                              pub_ip = "tcp://127.0.0.1:5999",
                              name="franklin", start_index=0)


agent_name = "franklin"
other_agent_name = "remus"

'''
Simulation of the NBEACONS domain (adapted from marsworld in [Dannenhauer and Munoz-Avila 2015]).

Notes: I will generate a state file instead of reading in from a file
'''

# Setup
thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
MIDCA_ROOT = thisDir + "/../" + "/../" + "/../"

### Domain Specific Variables
DOMAIN_ROOT = MIDCA_ROOT + "domains/grace/"
DOMAIN_FILE = DOMAIN_ROOT + "midcansf.sim"
STATE_FILE = DOMAIN_ROOT + "states/midcansf.sim" # state file is generated dynamically
DISPLAY_FUNC = nbeacons_util.drawNBeaconsScene
DECLARE_METHODS_FUNC = methods.declare_methods
DECLARE_OPERATORS_FUNC = operators.declare_ops
DECLARE_ACTIONS = asynch_grace
GOAL_GRAPH_CMP_FUNC = nbeacons_util.preferFree
DIMENSION_X = 5
DIMENSION_Y = 5



### Domain Specific Variables for JSHOP planner
JSHOP_DOMAIN_FILE = MIDCA_ROOT + "domains/grace/plan/midcaNsf.shp"
JSHOP_STATE_FILE = MIDCA_ROOT + "domains/grace/plan/midcansfproblem"+agent_name+".shp"

# percent chance each beacon will fail each tick
BEACON_FAIL_RATE = 0

# Load domain
world = domainread.load_domain(DOMAIN_FILE)

# Create Starting state
state1 = nbeacons_util.NBeaconGrid()
state1.generate(width=DIMENSION_X,height=DIMENSION_Y)
state1_str = state1.get_STRIPS_str()



# Load state
stateread.apply_state_file(world, STATE_FILE)
stateread.apply_state_str(world, state1_str)



#print (world)

# Creates a PhaseManager object, which wraps a MIDCA object
myMidca = base.PhaseManager(world, display=DISPLAY_FUNC, verbose=2)

# Add phases by name
for phase in ["Simulate", "Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
    myMidca.append_phase(phase)

# Add the modules which instantiate basic operation
myMidca.append_module("Simulate", simulator.ASCIIWorldViewer(DISPLAY_FUNC))
myMidca.append_module("Perceive", perceive.AsyncGraceObserver(interface, agent_name))
myMidca.append_module("Perceive", perceive.RecieveRequests(agent_name, other_agent_name))
myMidca.append_module("Perceive", perceive.HumanGoalInput(agent_name))
#myMidca.append_module("Perceive", perceive.GraceObserver())
#myMidca.append_module("Interpret", guide.GraceGoalInputNSF(interface))
myMidca.append_module("Interpret", guide.InterpretRequests(agent_name))
myMidca.append_module("Interpret", guide.GenerateRequests(agent_name, other_agent_name))
myMidca.append_module("Interpret", guide.EvaluateRequests(agent_name, other_agent_name))

myMidca.append_module("Interpret", guide.GraceAnomalyDetection(interface, agent_name, "smart"))
myMidca.append_module("Interpret", guide.GraceChangeDetection())
myMidca.append_module("Eval", evaluate.GraceReport(agent_name))
myMidca.append_module("Eval", evaluate.SimpleEvalAsync())
myMidca.append_module("Eval", evaluate.SimpleEvalSuspend())
#myMidca.append_module("Intend", intend.DfsIntendGraceNSF())
myMidca.append_module("Intend", intend.BestHillClimbingIntendGraceNSF(agent_name, "smart"))
myMidca.append_module("Intend", intend.PriorityIntend())
myMidca.append_module("Intend", intend.MetaIntendOps())
#myMidca.append_module("Intend", intend.SimpleIntendMultipleGoalsSuspend())
#myMidca.append_module("Intend", intend.SimpleIntend())
myMidca.append_module("Plan", planning.JSHOPPlanner(nbeacons_util.jshop2_state_from_world,
                                                        nbeacons_util.jshop2_tasks_from_goals,
                                                        JSHOP_DOMAIN_FILE,
                                                        JSHOP_STATE_FILE,
                                                        monitors= nbeacons_util.monitor
                                                    ))

myMidca.append_module("Plan", planning.PyHopPlanner(util.pyhop_state_from_world,
                                                    util.pyhop_tasks_from_goals,
                                                    DECLARE_METHODS_FUNC,
                                                    DECLARE_OPERATORS_FUNC))

myMidca.append_module("Act", act.SimpleAct(DECLARE_ACTIONS))

# Set world viewer to output text
myMidca.set_display_function(nbeacons_util.drawNBeaconsScene)

# Tells the PhaseManager to copy and store MIDCA states so they can be accessed later.
# Note: Turning this on drastically increases MIDCA's running time.
myMidca.storeHistory = False
myMidca.mem.logEachAccess = False

# Initialize and start running!
myMidca.init()
myMidca.initGoalGraph(cmpFunc = GOAL_GRAPH_CMP_FUNC)

def read_config(filename = "config.json"):
        '''
        input: filename
        :return: json object
        '''
        import json
        config = None
        with open(filename, "r") as f:
            config = json.load(f)
        return config

config = read_config()
agent_topic = config["agents"]["franklin"]["address"]["requests"]
interface_topic = config["publish_interface"]
zmqMidca = ZmqInterface.ZmqMidca(myMidca, incomingMsgHandlers = [
                                 handlers.RequestHandler(agent_topic, "bind", "sub", myMidca)],
                                 outgoingMsgHandlers = [
                                  handlers.OutReqHandler(interface_topic, "connect", "pub", myMidca),
                                 ])

zmqMidca.connect()
zmqMidca.run_midca(0.1)


#tag = TagWorldDemo.TagWorld()
#tag.move_cell([0,0], [0,0])

#myMidca.run(usingInterface=False)

