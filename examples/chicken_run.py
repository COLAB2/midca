from __future__ import print_function
import MIDCA
from MIDCA import base
from MIDCA.modules import simulator, perceive, guide, evaluate, intend, planning, act
from MIDCA.modules._plan import sample_methods, sample_operators
from MIDCA.worldsim import domainread, stateread


import inspect, os
'''
This script runs a simple version of MIDCA in a trivial domain where chickens cross the road.

https://github.com/mclumd/MIDCA/wiki/Running-Example-Scripts
'''

# Setup
thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
MIDCA_ROOT = thisDir + "/../"

# Load Domain Files  
domainFile = MIDCA_ROOT + "worldsim/domains/sample_domain.sim"
stateFile = MIDCA_ROOT + "worldsim/states/sample_state.sim"
world = domainread.load_domain(domainFile)
stateread.apply_state_file(world, stateFile)

# Creates a PhaseManager object, which wraps a MIDCA object
myMidca = base.PhaseManager(world, display = print, verbose=4)

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
myMidca.set_display_function(print) 

# Tells the PhaseManager to copy and store MIDCA states so they can be accessed later.
# Note: Turning this on drastically increases MIDCA's running time.
myMidca.storeHistory = False
myMidca.mem.logEachAccess = False

# Initialize and start running!
myMidca.init()
myMidca.run()
