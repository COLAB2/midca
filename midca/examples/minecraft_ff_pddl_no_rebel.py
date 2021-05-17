#!/usr/bin/env python
from midca import base,mem
from midca.modules import simulator, guide, evaluate, perceive, note, intend, planning, act
from midca.worldsim import stateread
from midca.worldsim import pddl_num_read as pddlread
#from midca.modules._plan.fastdown import testing_purp
#from midca.worldsim.DIRNAME import fast_downward
import inspect, os, pddlpy,random,shutil,fileinput
from midca.worldsim import pddl


# Domain Specific Imports
from midca.domains.ffdomain.minecraft import minecraft_util
# from midca.domains.minecraft import PDDL_util

thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
GOAL_GRAPH_CMP_FUNC = minecraft_util.preferknows

MIDCA_ROOT = thisDir + "/../"



### Domain Specific Variables
# DOMAIN_ROOT = MIDCA_ROOT + "domains/minecraft/"
# DOMAIN_FILE = DOMAIN_ROOT + "domains/domain.sim"
# STATE_FILE = DOMAIN_ROOT + "states/defstate.sim"

### Domain Specific Variables for JSHOP planner
#DOMAIN_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/domain.pddl"
#EVENT_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/domain_1.pddl"
#STATE_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/wood.1.pddl"
DISPLAY_FUNC = print

### this script is not working for now.
DOMAIN_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/plant-protection-actual.pddl"
EVENT_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/plant-protection-actual.pddl"
STATE_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/plant_pro_actual.pddl"
STATE_FILE_C = MIDCA_ROOT + "domains/ffdomain/minecraft/plant_pro_actual_2 (copy).pddl"
DOMAIN_FILE_P = MIDCA_ROOT + "domains/ffdomain/minecraft/plant-protection-modif.pddl"
EVENT_FILE_P = MIDCA_ROOT + "domains/ffdomain/minecraft/plant-protection-modif.pddl"
STATE_FILE_P = MIDCA_ROOT + "domains/ffdomain/minecraft/plant-pro_modif.pddl"

#inputfile = open(STATE_FILE_P,'r').readlines()
#for line in inputfile:
#    print(line)


a = list(range(6))
#b = random.choice(a[1:])
b = 4
print(a,b)
nat,inv = [],[]

for x in range(b+1):
    temp = [random.choice(a),random.choice(a)]
    while(temp in nat or temp in  inv):
        temp=[random.choice(a),random.choice(a)]
    nat.append(temp)
for x in range(b):
    temp1 = [random.choice(a),random.choice(a)]
    while(temp1 in nat or temp1 in  inv):
        temp1=[random.choice(a),random.choice(a)]
    inv.append(temp1)


state_file_2 = shutil.copyfile(STATE_FILE,STATE_FILE_C)
print(nat,inv)

inputfile = open(STATE_FILE_P,'r').readlines()
write_file = open(STATE_FILE_P,'w')
for line in inputfile:
    write_file.write(line)
    if '(and' in line:
        for i in range(b):
            native_plant = "(native-at pos%d-%d)" %(nat[i][0],nat[i][1])
            invasive_plant = "(invasive-at pos%d-%d)" %(inv[i][0],inv[i][1])
            write_file.write(native_plant + "\n")
            write_file.write(invasive_plant + "\n")
write_file.close()

inputfile = open(state_file_2,'r').readlines()
write_file = open(state_file_2,'w')
for line in inputfile:
    write_file.write(line)
    if ':init' in line:
        for i in range(b):
            native_plant = "(native-at pos%d-%d)" %(nat[i][0],nat[i][1])
            invasive_plant = "(invasive-at pos%d-%d)" %(inv[i][0],inv[i][1])
            request_1 = "(requested human1 agent1 native-at pos%d-%d )" %(nat[i][0],nat[i][1])
            request_2 = "(requested human1 agent1 not-invasive-at pos%d-%d )" %(inv[i][0],inv[i][1])
            write_file.write(native_plant + "\n")
            write_file.write(invasive_plant + "\n")
            write_file.write(request_1 + "\n")
            write_file.write(request_2 + "\n")
write_file.close()
#print(write_file)

#print(DOMAIN_FILE)
#DOMAIN_FILE = MIDCA_ROOT + "worldsim/dirname/plant-domain-mmm.pddl"
#EVENT_FILE = MIDCA_ROOT + "worldsim/dirname/plant-domain-mmm.pddl"
#STATE_FILE = MIDCA_ROOT + "worldsim/dirname/plant-mmmm.pddl"

#DOMAIN_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/domain_new.pddl"
#EVENT_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/domain_new.pddl"
#STATE_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/p01.pddl"

#world = pddlpy.DomainProblem(DOMAIN_FILE,STATE_FILE)
world = pddlread.load_domain(DOMAIN_FILE, state_file_2,EVENT_FILE)
world1= pddlread.load_domain(DOMAIN_FILE_P, STATE_FILE_C,EVENT_FILE_P)



'''
Simulation of tower construction and arson prevention in blocksworld. Uses
TF-trees and simulated Meta-AQUA connection to autonomously generate goals.
'''

'''thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
GOAL_GRAPH_CMP_FUNC = minecraft_util.preferknows

MIDCA_ROOT = thisDir + "/../"



### Domain Specific Variables
# DOMAIN_ROOT = MIDCA_ROOT + "domains/minecraft/"
# DOMAIN_FILE = DOMAIN_ROOT + "domains/domain.sim"
# STATE_FILE = DOMAIN_ROOT + "states/defstate.sim"

### Domain Specific Variables for JSHOP planner
#DOMAIN_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/domain.pddl"
#EVENT_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/domain_1.pddl"
#STATE_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/wood.1.pddl"
DISPLAY_FUNC = print

DOMAIN_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/plant-protection-actual.pddl"
EVENT_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/plant-protection-actual.pddl"
STATE_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/plant_pro_actual.pddl"

DOMAIN_FILE_P = MIDCA_ROOT + "domains/ffdomain/minecraft/plant-protection-modif.pddl"
EVENT_FILE_P = MIDCA_ROOT + "domains/ffdomain/minecraft/plant-protection-modif.pddl"
STATE_FILE_P = MIDCA_ROOT + "domains/ffdomain/minecraft/plant-pro_modif.pddl"
#print(DOMAIN_FILE)
#DOMAIN_FILE = MIDCA_ROOT + "worldsim/dirname/plant-domain-mmm.pddl"
#EVENT_FILE = MIDCA_ROOT + "worldsim/dirname/plant-domain-mmm.pddl"
#STATE_FILE = MIDCA_ROOT + "worldsim/dirname/plant-mmmm.pddl"

#DOMAIN_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/domain_new.pddl"
#EVENT_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/domain_new.pddl"
#STATE_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/p01.pddl"

#world = pddlpy.DomainProblem(DOMAIN_FILE,STATE_FILE)
world = pddlread.load_domain(DOMAIN_FILE, STATE_FILE,EVENT_FILE)'''
#world1= pddlread.load_domain(DOMAIN_FILE_P, STATE_FILE_P,EVENT_FILE_P)
#a= pddl.parseGoalDescription(world)
#print(a)

#stateread._apply_state_pddl(world, DOMAIN_FILE, STATE_FILE)
# creates a PhaseManager object, which wraps a MIDCA object
myMidca = base.PhaseManager(world, display='', verbose=4)

# add phases by name
for phase in ["Simulate", "Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
    myMidca.append_phase(phase)

# add the modules which instantiate basic blocksworld operation
myMidca.append_module("Simulate", simulator.MidcaActionSimulator())
#myMidca.append_module("Simulate", simulator.MidcaEventSimulator())
#myMidca.append_module("Simulate", simulator.ASCIIWorldViewer())
#myMidca.append_module("Perceive", perceive.PerfectObserver())
myMidca.append_module("Perceive", perceive.RemoteObserver())
#myMidca.append_module("Interpret", guide.UserGoalInput())
myMidca.append_module("Interpret", guide.MultipleGoalGen())
#myMidca.append_module("Interpret", note.GoalDiscrepancyDetector2(world1))

#myMidca.append_module("Interpret", note.GoalDiscrepancyDetector())
myMidca.append_module("Eval", evaluate.SimpleEvalSubgoals())
myMidca.append_module("Intend", intend.SimpleIntend())
#myMidca.append_module("Plan", planning.MetricFFPlanner(
   # minecraft_util.ff_goals_from_midca_goals,
   #minecraft_util.ff_state_from_midca_world,
    #DOMAIN_FILE,
    #STATE_FILE))
myMidca.append_module("Plan", planning.fastdownPlanner2(
   minecraft_util.ff_goals_from_midca_goals,
    minecraft_util.ff_state_from_midca_world,
    DOMAIN_FILE_P,
    STATE_FILE_P))
myMidca.append_module("Act", act.SimpleAct())
# myMidca.insert_module('Simulate', simulator.ArrowSimulator(arrowStart=1), 1)
# myMidca.insert_module('Simulate', simulator.AttackSimulator(skeletonStart=2), 1)
# myMidca.insert_module('Interpret', guide.TFFire(), 2)
#myMidca.insert_module('Interpret', guide.Informhuman(), 3)

# Set world viewer to output text
myMidca.set_display_function(DISPLAY_FUNC)


# tells the PhaseManager to copy and store MIDCA states so they can be accessed later.
myMidca.storeHistory = True
myMidca.initGoalGraph(cmpFunc=GOAL_GRAPH_CMP_FUNC)
myMidca.init()
for i in range(30):
    myMidca.one_cycle()
#myMidca.run()
#print(myMidca.mem.Goal_initial[0])
print(myMidca.mem.Goal_final_achieved_no_rebel)

'''
The code below would print out MIDCA's goal set for the first 20 phases of the run above. Note that any memory values can be accessed in this way, assuming that the storeHistory value was set to True during the run. This code is left as an example, but commented out because it will throw an error if fewer than 20 cycles were simulated.

for i in range(20):
    print myMidca.history[i].mem.get(myMidca.midca.mem.CURRENT_GOALS)

'''
#when the state changes I need to change the domain file for the planner :O
