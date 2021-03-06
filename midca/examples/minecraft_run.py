import midca
from midca import base
from midca.modules import simulator, guide, evaluate, perceive, intend, planning, act, note, assess
from midca.worldsim import domainread, stateread
import inspect, os

# Domain Specific Imports
from midca.domains.minecraft import minecraft_util
# TODO: This will have to be reworked when the file system is updated. I do not have access to my office and whiteboard drawings
from midca.domains.minecraft.plan import methods_minecraft, operators_minecraft  

'''
This template has been automatically created for your new domain, minecraft.
'''

# Setup
thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
MIDCA_ROOT = thisDir + "/../"

### Domain Specific Variables
DOMAIN_ROOT = MIDCA_ROOT + "domains/minecraft/"
# TODO: Next 2 lines will have to be reworked when the file system is updated. I do not have access to my office and whiteboard drawings
DOMAIN_FILE = DOMAIN_ROOT + "domain.sim"
STATE_FILE = DOMAIN_ROOT + "states/state.sim"
DISPLAY_FUNC = print
DECLARE_METHODS_FUNC = methods.declare_methods  # TODO: add your domain methods to midca/domains/minecraft/plan/methods.py
DECLARE_OPERATORS_FUNC = operators.declare_ops  # TODO: add your domain operators to midca/domains/minecraft/plan/operators.py


# Load Domain Files  
world = domainread.load_domain(DOMAIN_FILE)
stateread.apply_state_file(world, STATE_FILE)

# Creates a PhaseManager object, which wraps a MIDCA object
myMidca = base.PhaseManager(world, display = DISPLAY_FUNC, verbose=4)

# Add phases by name
for phase in ["Simulate", "Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
    myMidca.append_phase(phase)


# TODO: Update the module references to be domain specific functions
# Add the modules which instantiate basic operation
myMidca.append_module("Simulate", simulator.MidcaActionSimulator())
myMidca.append_module("Perceive", perceive.PerfectObserver())
myMidca.append_module("Interpret", guide.UserGoalInput())
myMidca.append_module("Eval", evaluate.SimpleEval())
myMidca.append_module("Intend", intend.SimpleIntend())
myMidca.append_module("Plan", planning.GenericPyhopPlanner(
    DECLARE_METHODS_FUNC, DECLARE_OPERATORS_FUNC)) # set up planner for sample domain
myMidca.append_module("Act", act.SimpleAct())

# Set world viewer with your specified DISPLAY_FUNC
myMidca.set_display_function(DISPLAY_FUNC) 

# Tells the PhaseManager to copy and store MIDCA states so they can be accessed later.
# Note: Turning this on drastically increases MIDCA's running time.
myMidca.storeHistory = False
myMidca.mem.logEachAccess = False

# Initialize and start running!
myMidca.init()
myMidca.run()