from __future__ import print_function
from midca import base
from midca.modules import simulator, perceive, guide,assess, evaluate, intend, planning, act
from midca.worldsim import domainread, stateread
from midca.domains.management import util
#from midca.domains.management.plan import moos_methods, moos_operators

# domain specific imports
from midca.domains.management.plan import management_methods, management_operators

import inspect, os

#Meta-Aqua
writePort = 5150
readPort = 5151

# Policy Count
POLICY_COUNT = 10
HIGHEST_SCORE = 100
INITIAL_SCORE = 70
RESOURCES = 500

# Setup
thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
MIDCA_ROOT = thisDir + "/../"

### Domain Specific Variables
DOMAIN_ROOT = MIDCA_ROOT + "domains/management/"
DOMAIN_FILE = DOMAIN_ROOT + "domain/management.sim"
STATE_FILE = DOMAIN_ROOT + "states/management_state.sim"
DISPLAY_FUNC = util.display
DECLARE_METHODS_FUNC = management_methods.declare_methods
DECLARE_OPERATORS_FUNC = management_operators.declare_ops
GOAL_GRAPH_CMP_FUNC = util.preferApprehend

# Load Domain Files
world = domainread.load_domain(DOMAIN_FILE)


state_str = open(STATE_FILE).read() # first read file

# now add new policys
for i in range(POLICY_COUNT):
    state_str+="POLICY(P"+str(i)+")\n"
    state_str+="available_policy(P"+str(i)+")\n"

# now add scores
for i in range(HIGHEST_SCORE+1):
    state_str+="SCORE("+str(i)+")\n"

# now add resources
for i in range(RESOURCES):
    state_str+="RESOURCES(R"+str(i+1)+")\n"

# now start with the initial reputation
state_str = state_str + "reputable(institution," + str(INITIAL_SCORE) +")\n"
state_str = state_str + "available_resources(R" + str(RESOURCES) +")\n"


# now load the state
stateread.apply_state_str(world, state_str)


# Creates a PhaseManager object, which wraps a MIDCA object
myMidca = base.PhaseManager(world, display = DISPLAY_FUNC, verbose=4)


# Add phases by name
for phase in ["Simulate","Perceive","Interpret","Eval","Intend","Plan","Act"]:
    myMidca.append_phase(phase)

# Add the modules which instantiate basic operation
myMidca.append_module("Simulate", simulator.ManagementActionSimulator())
myMidca.append_module("Simulate", simulator.ASCIIWorldViewer(display=DISPLAY_FUNC))
myMidca.append_module("Perceive",perceive.ManagementObserver(time=35))
myMidca.append_module("Interpret", guide.ManagementGoalInput(increment = 5))
myMidca.append_module("Eval", evaluate.SimpleEval())
myMidca.append_module("Intend", intend.SimpleIntend())

myMidca.append_module("Plan", planning.PyHopPlanner(util.pyhop_state_from_world,
                                                    util.pyhop_tasks_from_goals,
                                                    DECLARE_METHODS_FUNC,
                                                    DECLARE_OPERATORS_FUNC))

myMidca.append_module("Act", act.SimpleAct())
'''
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
myMidca.run(usingInterface=False)
