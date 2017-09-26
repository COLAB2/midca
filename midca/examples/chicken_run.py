from __future__ import print_function
from midca import base
from midca.modules import simulator, perceive, guide, evaluate, intend, planning, act
from midca.worldsim import domainread, stateread

# domain specific imports
from midca.domains.blocksworld.plan import sample_methods, sample_operators

import inspect, os
'''
This script runs a simple version of MIDCA in a trivial domain where chickens cross the road.

https://github.com/mclumd/MIDCA/wiki/Running-Example-Scripts
'''

# Setup
thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
MIDCA_ROOT = thisDir + "/../"

### Domain Specific Variables
DOMAIN_ROOT = MIDCA_ROOT + "domains/blocksworld/"
DOMAIN_FILE = DOMAIN_ROOT + "domains/sample_domain.sim"
STATE_FILE = DOMAIN_ROOT + "states/sample_state.sim"
DISPLAY_FUNC = print
DECLARE_METHODS_FUNC = sample_methods.declare_methods
DECLARE_OPERATORS_FUNC = sample_operators.declare_ops
GOAL_GRAPH_CMP_FUNC = None # not used in this example

# Load Domain Files  
world = domainread.load_domain(DOMAIN_FILE)
stateread.apply_state_file(world, STATE_FILE)

# Creates a PhaseManager object, which wraps a MIDCA object
myMidca = base.PhaseManager(world, display = DISPLAY_FUNC, verbose=4)

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
    DECLARE_METHODS_FUNC, DECLARE_OPERATORS_FUNC)) # set up planner for sample domain
myMidca.append_module("Act", act.SimpleAct())

# Set world viewer to output text
myMidca.set_display_function(DISPLAY_FUNC) 

# Tells the PhaseManager to copy and store MIDCA states so they can be accessed later.
# Note: Turning this on drastically increases MIDCA's running time.
myMidca.storeHistory = False
myMidca.mem.logEachAccess = False

# Initialize and start running!
myMidca.init()
myMidca.run()
