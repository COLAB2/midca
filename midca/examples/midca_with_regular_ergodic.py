#!/usr/bin/env python
import midca
from midca import base
from midca.modules import simulator, guide, evaluate, perceive, intend, planning, act, note, assess
from midca.worldsim import domainread, stateread
from midca.modules._plan.asynch import asynch_grace
import inspect, os
import threading

# Domain Specific Imports
from midca.domains.nbeacons import grace_util as nbeacons_util
from midca.domains.nbeacons.plan import methods_nbeacons, operators_nbeacons

# simulator
from midca.domains.nbeacons.tagsim import TagWorldDemo

#run simulator
tag = TagWorldDemo.TagWorld()
tag.runSim()

tag = TagWorldDemo.TagWorld()
tag.runSim()

#tag = TagWorldDemo.TagWorld()
#tag.move_cell([0,0], [0,0])

#tag = TagWorldDemo.TagWorld()
#tag.move_cell([0,0], [3,4])

'''
Simulation of the NBEACONS domain (adapted from marsworld in [Dannenhauer and Munoz-Avila 2015]).

Notes: I will generate a state file instead of reading in from a file
'''

# Setup
thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
MIDCA_ROOT = thisDir + "/../"

### Domain Specific Variables
DOMAIN_ROOT = MIDCA_ROOT + "domains/nbeacons/"
DOMAIN_FILE = DOMAIN_ROOT + "domains/midcansf.sim"
STATE_FILE = DOMAIN_ROOT + "states/midcansf.sim" # state file is generated dynamically
DISPLAY_FUNC = nbeacons_util.drawNBeaconsScene
DECLARE_METHODS_FUNC = methods_nbeacons.declare_methods
DECLARE_OPERATORS_FUNC = operators_nbeacons.declare_operators
GOAL_GRAPH_CMP_FUNC = nbeacons_util.preferFree
DIMENSION = 5


### Domain Specific Variables for JSHOP planner
JSHOP_DOMAIN_FILE = MIDCA_ROOT + "domains/nbeacons/plan/midcaNsf.shp"
JSHOP_STATE_FILE = MIDCA_ROOT + "domains/nbeacons/plan/midcansfproblem.shp"

# percent chance each beacon will fail each tick
BEACON_FAIL_RATE = 0

# Load domain
world = domainread.load_domain(DOMAIN_FILE)

# Create Starting state
state1 = nbeacons_util.NBeaconGrid()
state1.generate(width=DIMENSION,height=DIMENSION)
state1_str = state1.get_STRIPS_str()


# Load state
stateread.apply_state_file(world, STATE_FILE)
stateread.apply_state_str(world, state1_str)

#method
#method = "susd"
method = "tags"

# Creates a PhaseManager object, which wraps a MIDCA object
myMidca = base.PhaseManager(world, display=DISPLAY_FUNC, verbose=2)

# Add phases by name
for phase in ["Simulate", "Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
    myMidca.append_phase(phase)

# Add the modules which instantiate basic operation
myMidca.append_module("Simulate", simulator.ASCIIWorldViewer(DISPLAY_FUNC))
if method == "tags":
    myMidca.append_module("Perceive", perceive.AsyncGraceObserver())
else:
    myMidca.append_module("Perceive", perceive.AsyncGraceObserverSusd())
#myMidca.append_module("Perceive", perceive.GraceObserver())
#myMidca.append_module("Interpret", guide.GraceGoalInputNSF())
myMidca.append_module("Interpret", guide.GraceAnomalyDetection())
myMidca.append_module("Eval", evaluate.SimpleEvalAsync())
#myMidca.append_module("Intend", intend.DfsIntendGraceNSF())
#myMidca.append_module("Intend", intend.BestHillClimbingIntendGraceNSF())
#myMidca.append_module("Intend", intend.PriorityIntend())
myMidca.append_module("Intend", intend.SimpleIntend())
myMidca.append_module("Plan", planning.JSHOPPlannerAsync(nbeacons_util.jshop2_state_from_world,
                                                        nbeacons_util.jshop2_tasks_from_goals,
                                                        JSHOP_DOMAIN_FILE,
                                                        JSHOP_STATE_FILE,
                                                        asynch_grace,
                                                        monitors= nbeacons_util.monitor
                                                    ))
myMidca.append_module("Act", act.AsynchronousGraceAct())


# Set world viewer to output text
myMidca.set_display_function(nbeacons_util.drawNBeaconsScene)

# Tells the PhaseManager to copy and store MIDCA states so they can be accessed later.
# Note: Turning this on drastically increases MIDCA's running time.
myMidca.storeHistory = False
myMidca.mem.logEachAccess = False

# Initialize and start running!
myMidca.init()
myMidca.initGoalGraph(cmpFunc = GOAL_GRAPH_CMP_FUNC)

#tag = TagWorldDemo.TagWorld()
#tag.move_cell([0,0], [0,0])

myMidca.run(usingInterface=False)
