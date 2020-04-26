from __future__ import print_function
from midca import base
from midca.modules import simulator, perceive, guide,assess, evaluate, intend, planning, act
from midca.worldsim import domainread, stateread
from midca.domains.moos_domain import util, moosworld, experiment
from midca.domains.moos_domain.plan import moos_methods, moos_operators
import os

# domain specific imports
#from midca.domains.moos_domain.plan import sample_methods, sample_operators

import inspect, os, time
import os.path


# Setup
thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
MIDCA_ROOT = thisDir + "/../"

### Domain Specific Variables
DOMAIN_ROOT = MIDCA_ROOT + "domains/moos_domain/"
DOMAIN_FILE = DOMAIN_ROOT + "domain/moos_flairs.sim"
STATE_FILE = DOMAIN_ROOT + "states/moos_flairs.sim"
DISPLAY_FUNC = util.display
DECLARE_METHODS_FUNC = moos_methods.declare_methods
DECLARE_OPERATORS_FUNC = moos_operators.declare_ops
GOAL_GRAPH_CMP_FUNC = util.preferApprehend

# Load Domain Files
world = domainread.load_domain(DOMAIN_FILE)
stateread.apply_state_file(world, STATE_FILE)

# Creates a PhaseManager object, which wraps a MIDCA object
myMidca = base.PhaseManager(world, display = DISPLAY_FUNC, verbose=4)


#mooos specifics
index = 0
index_copy = 0
f = open(MIDCA_ROOT + "/examples/range.txt", "r")
index = int(f.readline())
f.close()


DIR = MIDCA_ROOT + '/examples/results/'
file_count = len([name for name in os.listdir(DIR) if os.path.isfile(os.path.join(DIR, name))])

if file_count == 0 or index ==0:
        with open(DIR+str(file_count)+'.csv','a') as fd:
            pass
        file_count +=1

deadline= []
for i in range(0, 220, 50):
    deadline.append(i)



if (len(deadline)) == (index+1):
    index_copy = index
    index = -1

f = open(MIDCA_ROOT + "/examples/range.txt", "w")
f.write(str(index+1))
f.close()

if index == -1:
    index = index_copy

#--------------------------------

# Add phases by name
for phase in ["Simulate","Perceive","Interpret","Eval","Intend","Plan","Act"]:
    myMidca.append_phase(phase)
# Add the modules which instantiate basic operation
#myMidca.append_module("Simulate", simulator.MoosWorldViewer(display=DISPLAY_FUNC))
myMidca.append_module("Perceive",perceive.MoosObserverMultiagent())
myMidca.append_module("Interpret", guide.MoosGoalInput(deadline=deadline[index]+ 1085))
#myMidca.append_module("Interpret", guide.MoosGoalInterpretFlairs())
myMidca.append_module("Eval", evaluate.EvalMoosFromFeedback(DIR, deadline[index]))
myMidca.append_module("Intend", intend.SimpleIntendFlairs(deadline[index], index, file_count))
myMidca.append_module("Plan", planning.PyHopPlannerMoos(util.pyhop_state_from_world,
                                                    util.pyhop_tasks_from_goals,
                                                    DECLARE_METHODS_FUNC,
                                                    DECLARE_OPERATORS_FUNC)) # set up planner for sample domain
myMidca.append_module("Act", act.AsynchronousMoosAct())


"""
myMidca.append_module("Interpret", guide.MoosGoalInput(deadline=250))
myMidca.append_module("Interpret", guide.MoosGoalInterpret())
myMidca.append_module("Eval", evaluate.SimpleEval_moos())
myMidca.append_module("Intend", intend.SimpleIntend())
myMidca.append_module("Plan", planning.PyHopPlanner(util.pyhop_state_from_world,
                                                    util.pyhop_tasks_from_goals,
                                                    DECLARE_METHODS_FUNC,
                                                    DECLARE_OPERATORS_FUNC)) # set up planner for sample domain
myMidca.append_module("Act", act.Moosact())
"""


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



# moos specific experimental functions
e = experiment.MultiExperiment()
e.lay_mines(file_count-1)
moosworld.main(deadline[index])
myMidca.run(usingInterface=False)

