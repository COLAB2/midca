#!/usr/bin/env python
import MIDCA
from MIDCA import base, goals
from MIDCA.modules import simulator, guide, evaluate, perceive, intend, planning, act, note, assess
from MIDCA.metamodules import monitor, control, interpret, metaintend, plan
from MIDCA.worldsim import domainread, stateread
import inspect, os
import random

# Domain Specific Imports
from MIDCA.domains.nbeacons import nbeacons_util
from MIDCA.domains.nbeacons.plan import methods_nbeacons, operators_nbeacons

'''
Simulation of the NBEACONS domain (adapted from marsworld in [Dannenhauer and Munoz-Avila 2015]).

THIS IS THE START SCRIPT FOR THE META COGNITIVE AGENT (Agent 3)

'''

wind_schedule = [[10,1],[50,2],[120,3],[200,4]]
# generate goals randomly, such that no goal is repeated or occurs in the last 3 goals
num_goals = 100
goal_list = []
i = 0
possible_goals = range(10)
last_chosen_goal = -1
while i < num_goals:
    if last_chosen_goal == -1:
        curr_goal = random.choice(possible_goals)
        goal_list.append(curr_goal)
        last_chosen_goal = curr_goal
    else:
        tmp_possible_goals = set(possible_goals) - set([last_chosen_goal])
        curr_goal = random.sample(tmp_possible_goals,1)[0]
        goal_list.append(curr_goal)
        last_chosen_goal = curr_goal
    i+=1
        
goal_list = map(lambda x: goals.Goal('B'+str(x), predicate = "activated"), goal_list)

# Setup
thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
MIDCA_ROOT = thisDir + "/../"

### Domain Specific Variables
DOMAIN_ROOT = MIDCA_ROOT + "domains/nbeacons/"
DOMAIN_FILE = DOMAIN_ROOT + "domains/nbeacons_avoid_mud.sim"
#STATE_FILE = DOMAIN_ROOT + "states/.sim" # state file is generated dynamically
DISPLAY_FUNC = nbeacons_util.drawNBeaconsScene
DECLARE_METHODS_FUNC = methods_nbeacons.declare_methods
DECLARE_OPERATORS_FUNC = operators_nbeacons.declare_operators
GOAL_GRAPH_CMP_FUNC = nbeacons_util.preferFree

DIMENSION = 16 # width and height of grid
BEACON_FAIL_RATE = 20 # percent chance each beacon will fail each tick
WIND_ENABLED = True 
WIND_DIR = 'east' # direction to push the agent if it moves in this direction
WIND_STRENGTH = 0 # number of extra tiles for the agent to move
NUM_QUICKSAND = 10

# Load domain
world = domainread.load_domain(DOMAIN_FILE)

# Create Starting state
state1 = nbeacons_util.NBeaconGrid()
#state1.generate_good_test()
state1.generate(width=DIMENSION,height=DIMENSION,num_beacons=10,num_quicksand_spots=NUM_QUICKSAND)
state1_str = state1.get_STRIPS_str()

# Load state
stateread.apply_state_str(world, state1_str)

# Creates a PhaseManager object, which wraps a MIDCA object
myMidca = base.PhaseManager(world, display=DISPLAY_FUNC, verbose=2, metaEnabled=True)

# Add phases by name
for phase in ["Simulate", "Perceive", "Interpret1", "Interpret2", "Interpret3", "Eval", "Cleanup", "Intend", "Plan", "Act"]:
    myMidca.append_phase(phase)

# Add the modules which instantiate basic operation
#myMidca.append_module("Simulate", simulator.MidcaActionSimulator())
myMidca.append_module("Simulate", simulator.NBeaconsActionSimulator(wind=WIND_ENABLED,wind_dir=WIND_DIR,wind_strength=WIND_STRENGTH,dim=DIMENSION,wind_schedule=wind_schedule))

myMidca.append_module("Simulate", simulator.ASCIIWorldViewer(DISPLAY_FUNC))

myMidca.append_module("Perceive", perceive.PerfectObserver())

myMidca.append_module("Interpret1", note.StateDiscrepancyDetector())
myMidca.append_module("Interpret2", assess.SimpleNBeaconsExplain())
myMidca.append_module("Interpret3", guide.SimpleNBeaconsGoalManager())
#myMidca.append_module("Interpret", assess.SimpleNBeaconsExplain())
#myMidca.append_module("Interpret", assess.SimpleNBeaconsExplain())

#myMidca.append_module("Interpret", guide.UserGoalInput())
myMidca.append_module("Interpret3", guide.NBeaconsGoalGenerator(numbeacons=2,goalList=goal_list))
myMidca.append_module("Eval", evaluate.NBeaconsDataRecorder())
myMidca.append_module("Cleanup", simulator.NBeaconsSimulator(beacon_fail_rate=BEACON_FAIL_RATE))
myMidca.append_module("Intend", intend.SimpleIntend())
myMidca.append_module("Plan", planning.HeuristicSearchPlanner())
#myMidca.append_module("Plan", planning.PyHopPlanner(nbeacons_util.pyhop_state_from_world,
#                                                    nbeacons_util.pyhop_tasks_from_goals,
#                                                    DECLARE_METHODS_FUNC,
#                                                    DECLARE_OPERATORS_FUNC)) # set up planner for sample domain
myMidca.append_module("Act", act.NBeaconsSimpleAct())

for phase in ["Monitor", "Interpret", "Intend", "Plan", "Control"]:
            myMidca.append_meta_phase(phase)
        
# add meta layer modules
myMidca.append_meta_module("Monitor", monitor.MRSimpleMonitor())
myMidca.append_meta_module("Interpret", interpret.MRSimpleDetect())
#myMidca.append_meta_module("Interpret", interpret.MRSimpleGoalGenForGoalTrans())
myMidca.append_meta_module("Intend", metaintend.MRSimpleIntend())
myMidca.append_meta_module("Plan", plan.MRSimplePlanner())
myMidca.append_meta_module("Control", control.MRSimpleControl())

# Set world viewer to output text
myMidca.set_display_function(nbeacons_util.drawNBeaconsScene) 

# Tells the PhaseManager to copy and store MIDCA states so they can be accessed later.
# Note: Turning this on drastically increases MIDCA's running time.
myMidca.storeHistory = False
myMidca.mem.logEachAccess = False

# Initialize and start running!
myMidca.init()
myMidca.initGoalGraph(cmpFunc = GOAL_GRAPH_CMP_FUNC)
myMidca.run()
