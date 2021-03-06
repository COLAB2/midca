#!/usr/bin/env python
import midca
from midca import base
from midca.modules.perceive import PerfectObserver
from midca.modules.plan import HeuristicSearchPlanner
from midca.modules.intend import SimpleIntend
from midca.modules.act import SimpleAct
from midca.modules.interpret import NBeaconsGoalGenerator, SimpleNBeaconsExplain, UserGoalInput, \
    StateDiscrepancyDetector
from midca.modules.evaluate import NBeaconsDataRecorder
from midca.modules import simulator
from midca.worldsim import domainread, stateread
import inspect, os

# Domain Specific Imports
from midca.domains.nbeacons import nbeacons_util
from midca.domains.nbeacons.plan import methods_nbeacons, operators_nbeacons

'''
Simulation of the NBEACONS domain (adapted from marsworld in [Dannenhauer and Munoz-Avila 2015]).

Some examples of goals to give during the interpret phase, when prompted for a goal:

agent-at(Curiosity, Tx3y7)
...
agent-at(Curiosity, Tx15y2)

'''

# Setup
thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
MIDCA_ROOT = thisDir + "/../"

### Domain Specific Variables
DOMAIN_ROOT = MIDCA_ROOT + "domains/nbeacons/"
DOMAIN_FILE = DOMAIN_ROOT + "nbeacons.sim"
#STATE_FILE = DOMAIN_ROOT + "states/.sim" # state file is generated dynamically
DISPLAY_FUNC = nbeacons_util.drawNBeaconsScene
DECLARE_METHODS_FUNC = methods_nbeacons.declare_methods
DECLARE_OPERATORS_FUNC = operators_nbeacons.declare_operators
GOAL_GRAPH_CMP_FUNC = None
DIMENSION = 20

# percent chance each beacon will fail each tick
BEACON_FAIL_RATE = 0

# Load domain
world = domainread.load_domain(DOMAIN_FILE)

# Create Starting state
state1 = nbeacons_util.NBeaconGrid()
state1.generate(width=DIMENSION,height=DIMENSION,num_beacons=10,num_quicksand_spots=0)
state1_str = state1.get_STRIPS_str()

# Load state
stateread.apply_state_str(world, state1_str)

# Creates a PhaseManager object, which wraps a MIDCA object
myMidca = base.PhaseManager(world, display=DISPLAY_FUNC, verbose=2)

# Add phases by name
for phase in ["Simulate", "Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
    myMidca.append_phase(phase)

# Add the modules which instantiate basic operation
#myMidca.append_module("Simulate", simulator.MidcaActionSimulator())
myMidca.append_module("Simulate", simulator.NBeaconsActionSimulator(wind=False,wind_dir='off',dim=DIMENSION))
#myMidca.append_module("Simulate", simulator.NBeaconsSimulator(beacon_fail_rate=BEACON_FAIL_RATE))
myMidca.append_module("Simulate", simulator.ASCIIWorldViewer(DISPLAY_FUNC))
myMidca.append_module("Perceive", PerfectObserver.PerfectObserver())

myMidca.append_module("Interpret", StateDiscrepancyDetector.StateDiscrepancyDetector())
myMidca.append_module("Interpret", SimpleNBeaconsExplain.SimpleNBeaconsExplain())
myMidca.append_module("Interpret", UserGoalInput.UserGoalInput())
myMidca.append_module("Eval", NBeaconsDataRecorder.NBeaconsDataRecorder())
myMidca.append_module("Intend", SimpleIntend.SimpleIntend())
myMidca.append_module("Plan", HeuristicSearchPlanner.HeuristicSearchPlanner())
#myMidca.append_module("Plan", planning.PyHopPlanner(nbeacons_util.pyhop_state_from_world,
#                                                    nbeacons_util.pyhop_tasks_from_goals,
#                                                    DECLARE_METHODS_FUNC,
#                                                    DECLARE_OPERATORS_FUNC)) # set up planner for sample domain
myMidca.append_module("Act", SimpleAct.SimpleAct())

# Set world viewer to output text
myMidca.set_display_function(nbeacons_util.drawNBeaconsScene) 

# Tells the PhaseManager to copy and store MIDCA states so they can be accessed later.
# Note: Turning this on drastically increases MIDCA's running time.
myMidca.storeHistory = False
myMidca.mem.logEachAccess = False

# Initialize and start running!
myMidca.init()
myMidca.initGoalGraph()
myMidca.run()
