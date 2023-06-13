#!/usr/bin/env python
import midca
from midca import base
from midca.modules import simulator, guide, evaluate, perceive, intend, planning, act, note, assess
from midca.worldsim import domainread, stateread
from midca.modules._plan.asynch import asynch_grace
import inspect, os, sys
import threading

# Domain Specific Imports
from midca.domains.grace import grace_util as nbeacons_util
# simulator
from midca.domains.grace.interface import tagworld

# Domain Specific Imports
from midca.domains.blocksworld import util
from midca.domains.blocksworld.plan import methods_multiAgent as methods, operators_multiAgent as operators

agent_name = "grace"


'''
Simulation of the NBEACONS domain (adapted from marsworld in [Dannenhauer and Munoz-Avila 2015]).

Notes: I will generate a state file instead of reading in from a file
'''


# Setup
thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
MIDCA_ROOT = thisDir + "/../"

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
JSHOP_STATE_FILE = MIDCA_ROOT + "domains/grace/plan/midcansfproblem.shp"

# percent chance each beacon will fail each tick
BEACON_FAIL_RATE = 0

# Load domain
world = domainread.load_domain(DOMAIN_FILE)

# Create Starting state
state1 = nbeacons_util.NBeaconGrid()
state1.generate(width=DIMENSION_X,height=DIMENSION_Y)
state1_str = state1.get_STRIPS_str()

state1_str += "agent-at(grace, Tx2y2)\n"
#state1_str += "agent-at(remus, Tx2y3)\n"

# Load state
stateread.apply_state_file(world, STATE_FILE)
stateread.apply_state_str(world, state1_str)

# Creates a PhaseManager object, which wraps a MIDCA object
myMidca = base.PhaseManager(world, display=DISPLAY_FUNC, verbose=2)

# Add phases by name
for phase in ["Simulate", "Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
    myMidca.append_phase(phase)

# Add the modules which instantiate basic operation
myMidca.append_module("Simulate", simulator.ASCIIWorldViewer(DISPLAY_FUNC))

# Set world viewer to output text
myMidca.set_display_function(nbeacons_util.drawNBeaconsScene)

# Tells the PhaseManager to copy and store MIDCA states so they can be accessed later.
# Note: Turning this on drastically increases MIDCA's running time.
myMidca.storeHistory = False
myMidca.mem.logEachAccess = True

# Initialize and start running!
myMidca.init()
myMidca.initGoalGraph(cmpFunc = GOAL_GRAPH_CMP_FUNC)
myMidca.run(usingInterface=True)
