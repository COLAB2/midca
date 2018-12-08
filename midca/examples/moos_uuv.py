from __future__ import print_function
from midca import base
from midca.modules import simulator, perceive, guide,assess, evaluate, intend, planning, act
from midca.worldsim import domainread, stateread
from midca.domains.moos_domain import util
from midca.domains.moos_domain.plan import moos_methods, moos_operators

# domain specific imports
#from midca.domains.moos_domain.plan import sample_methods, sample_operators

import inspect, os
'''
This script runs a simple version of MIDCA in a trivial domain where chickens cross the road.

https://github.com/mclumd/MIDCA/wiki/Running-Example-Scripts
'''

#Meta-Aqua
writePort = 5150
readPort = 5151

# Setup
thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
MIDCA_ROOT = thisDir + "/../"

### Domain Specific Variables
DOMAIN_ROOT = MIDCA_ROOT + "domains/moos_domain/"
DOMAIN_FILE = DOMAIN_ROOT + "domain/moos_domain.sim"
STATE_FILE = DOMAIN_ROOT + "states/moos_state.sim"
DISPLAY_FUNC = util.display
DECLARE_METHODS_FUNC = moos_methods.declare_methods
DECLARE_OPERATORS_FUNC = moos_operators.declare_ops
GOAL_GRAPH_CMP_FUNC = util.preferApprehend

# Load Domain Files
world = domainread.load_domain(DOMAIN_FILE)
stateread.apply_state_file(world, STATE_FILE)

# Creates a PhaseManager object, which wraps a MIDCA object
myMidca = base.PhaseManager(world, display = DISPLAY_FUNC, verbose=4)

# Add phases by name
for phase in ["Simulate","Perceive","Interpret","Eval","Intend","Plan","Act"]:
    myMidca.append_phase(phase)
# Add the modules which instantiate basic operation
myMidca.append_module("Simulate", simulator.ASCIIWorldViewer(display=DISPLAY_FUNC))
myMidca.append_module("Perceive",perceive.MoosObserver())
myMidca.append_module("Interpret", guide.MoosGoalInput(deadline=250))
myMidca.append_module("Eval", evaluate.SimpleEval_moos())
myMidca.append_module("Intend", intend.SimpleIntend())
myMidca.append_module("Plan", planning.PyHopPlanner(util.pyhop_state_from_world,
                                                    util.pyhop_tasks_from_goals,
                                                    DECLARE_METHODS_FUNC,
                                                    DECLARE_OPERATORS_FUNC)) # set up planner for sample domain
myMidca.append_module("Act", act.Moosact())



'''
# Set world viewer to output text
myMidca.set_display_function(DISPLAY_FUNC)

# Tells the PhaseManager to copy and store MIDCA states so they can be accessed later.
# Note: Turning this on drastically increases MIDCA's running time.
myMidca.storeHistory = False
myMidca.mem.logEachAccess = False
'''
# Initialize and start running!
myMidca.initGoalGraph(cmpFunc = GOAL_GRAPH_CMP_FUNC)
myMidca.init()
myMidca.run()
