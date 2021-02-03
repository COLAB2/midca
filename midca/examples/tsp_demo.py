#!/usr/bin/env python
import midca
from midca import base
from midca.modules import simulator, guide, evaluate, perceive, intend, planning, act, note, assess
from midca.worldsim import domainread, stateread
import inspect, os

# Domain Specific Imports
from midca.domains.nbeacons import nbeacons_util
from midca.domains.nbeacons.plan import methods_nbeacons, operators_nbeacons

'''
Simulation of the Traveling Salesman domain (from https://github.com/SoarGroup/Domains-Planning-Domain-Definition-Language/blob/master/pddl/tsp-10.pddl)

A good goal to give to the agent during the interpret phase:

visited(c1)
visited(c2)
visited(c3)
visited(c4)
visited(c5)
visited(c6)
visited(c7)
visited(c8)
visited(c9)
visited(c10)
complete()

'''

# Setup
thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
MIDCA_ROOT = thisDir + "/../"

### Domain Specific Variables
DOMAIN_ROOT = MIDCA_ROOT + "domains/tsp/"
DOMAIN_FILE = DOMAIN_ROOT + "domains/tsp.sim"
STATE_FILE = DOMAIN_ROOT + "states/ten_cities_tsp.sim"
DISPLAY_FUNC = None
#DECLARE_METHODS_FUNC = methods_nbeacons.declare_methods
#DECLARE_OPERATORS_FUNC = operators_nbeacons.declare_operators
GOAL_GRAPH_CMP_FUNC = None
DIMENSION = 20

# percent chance each beacon will fail each tick
BEACON_FAIL_RATE = 0

# Load domain
world = domainread.load_domain(DOMAIN_FILE)

# Load state
stateread.apply_state_file(world, STATE_FILE)

# Creates a PhaseManager object, which wraps a MIDCA object
myMidca = base.PhaseManager(world, display=DISPLAY_FUNC, verbose=2)

# Add phases by name
for phase in ["Simulate", "Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
    myMidca.append_phase(phase)

# Add the modules which instantiate basic operation
myMidca.append_module("Simulate", simulator.MidcaActionSimulator())
myMidca.append_module("Simulate", simulator.ASCIIWorldViewer(DISPLAY_FUNC))
myMidca.append_module("Perceive", perceive.PerfectObserver())

myMidca.append_module("Interpret", guide.UserGoalInput())
myMidca.append_module("Eval", evaluate.SimpleEval())
myMidca.append_module("Intend", intend.SimpleIntend())
myMidca.append_module("Plan", planning.HeuristicSearchPlanner())

myMidca.append_module("Act", act.SimpleAct())

# Set world viewer to output text
#myMidca.set_display_function(nbeacons_util.drawNBeaconsScene)

# Tells the PhaseManager to copy and store MIDCA states so they can be accessed later.
# Note: Turning this on drastically increases MIDCA's running time.
myMidca.storeHistory = False
myMidca.mem.logEachAccess = False

# Initialize and start running!
myMidca.init()
myMidca.initGoalGraph()
myMidca.run()
