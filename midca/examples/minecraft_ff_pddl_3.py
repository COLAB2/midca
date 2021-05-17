'''
Created on Jul 16, 2015

@author: dustin
'''

from midca import base
from midca.modules import simulator, guide, evaluate, perceive, note, intend, planning, act
from midca.worldsim import stateread
from midca.worldsim import pddl_num_read as pddlread
#from midca.modules._plan.fastdown import testing_purp
#from midca.worldsim.DIRNAME import fast_downward
import inspect, os, pddlpy,random,shutil,fileinput
from midca.worldsim import pddl


# Domain Specific Imports
from midca.domains.ffdomain.minecraft import minecraft_util
from midca.experiment.experiment import Experiment
from midca import base
from functools import partial

thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
GOAL_GRAPH_CMP_FUNC = minecraft_util.preferknows

MIDCA_ROOT = thisDir + "/../"

DOMAIN_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/plant-protection-actual.pddl"
EVENT_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/plant-protection-actual.pddl"
STATE_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/plant_pro_actual.pddl"
STATE_FILE_C = MIDCA_ROOT + "domains/ffdomain/minecraft/plant_pro_actual_2 (copy).pddl"
DOMAIN_FILE_P = MIDCA_ROOT + "domains/ffdomain/minecraft/plant-protection-modif.pddl"
EVENT_FILE_P = MIDCA_ROOT + "domains/ffdomain/minecraft/plant-protection-modif.pddl"
STATE_FILE_P = MIDCA_ROOT + "domains/ffdomain/minecraft/plant-pro_modif.pddl"
DATA_FILENAME = MIDCA_ROOT + "domains/ffdomain/minecraft/sas_plan"





class ArsonCogSciDemo():
    '''
    This is a custom experiment where MIDCA is run by varying the following:
    - probability of fire
    - using TFTrees for putoutfire goals
    - using simulated metaaqua to generate goal to put out arsonist
    '''

    def __init__(self):
        '''
        Setup code for experiment
        '''
        thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
        GOAL_GRAPH_CMP_FUNC = minecraft_util.preferknows

        MIDCA_ROOT = thisDir + "/../"

        DOMAIN_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/plant-protection-actual.pddl"
        EVENT_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/plant-protection-actual.pddl"
        STATE_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/plant_pro_actual.pddl"
        STATE_FILE_C = MIDCA_ROOT + "domains/ffdomain/minecraft/plant_pro_actual_2 (copy).pddl"
        DOMAIN_FILE_P = MIDCA_ROOT + "domains/ffdomain/minecraft/plant-protection-modif.pddl"
        EVENT_FILE_P = MIDCA_ROOT + "domains/ffdomain/minecraft/plant-protection-modif.pddl"
        STATE_FILE_P = MIDCA_ROOT + "domains/ffdomain/minecraft/plant-pro_modif.pddl"
        DATA_FILENAME = MIDCA_ROOT + "domains/ffdomain/minecraft/sas_plan"

        # initialize the csv file here
        file = open(DATA_FILENAME, 'a')
        file.write("runID,arsonchance,usingTFTree,usingSimMA,towersCompleted,score,\n")

        #def customWriteData(run_id, midca):
        '''
            This function will be called on each midca object after it has run and should get the
            necessary values (to be written to a data file) from the midca object.

            Customize this function to retrieve the data you wish to collect during the experiment
            '''
        # get the arson chance from the module
        '''arsonchance = -1
            for module in midca.get_modules("Simulate"):
                if module.__class__.__name__ == "ArsonSimulator":
                    arsonchance = module.getArsonChance()

            # determine if TFTrees are being used
            usingTFTree = False
            for module in midca.get_modules("Interpret"):
                if module.__class__.__name__ == "TFFire":
                    usingTFTree = True

            # determine if simulateMA is being used
            usingSimMA = False
            for module in midca.get_modules("Interpret"):
                if module.__class__.__name__ == "ReactiveApprehend":
                    usingSimMA = True

            # get tower score
            towersCompleted = midca.mem.get(evaluate.SCORE).getTowersCompleted()
            towersScore = midca.mem.get(evaluate.SCORE).getTowersScore()
            print()
            str(midca.mem.get(evaluate.SCORE))'''

            #file.write(
            #    str(run_id) + "," + str(arsonchance) + "," + str(usingTFTree) + "," + str(usingSimMA) + "," + str(
            #        towersCompleted) + "," + str(towersScore) + "\n")
            #file.flush()

        #def cleanup():
        '''
            This function is used to execute any code that needs to be run
            before closing, (i.e. closing file handles)
            '''
            #file.close()

        NUM_CYCLES = 75

        ###### create code for each MIDCA run #####
        ex = Experiment(self.__class__.__name__)

        #ex.addWriteDataFunc(customWriteData)
        #ex.addDestructFunc(cleanup)

        ex.setNumCycles(NUM_CYCLES)


        # since we are varying using 3 parameters (arson chance, using tf trees, using MA)
        # we have three nested loops, creating individual midca runs for each unique paramterization

        print()
        "Initializing each unique MIDCA run..."
        # time.sleep(0.5)
        # 1. vary by arson chance
        '''curr_arson_chance = ARSON_CHANCE_START
        while curr_arson_chance > ARSON_CHANCE_END:

            # 2. vary by using TF Trees
            for useTFTrees in [True, False]:

                # 3. vary using MA (Simulated version)
                for useSimMA in [True, False]:
                    # create MIDCA instance
                    midcaInst = MIDCAInstance(curr_arson_chance, useTFTrees, useSimMA)
                    midcaInst.createMIDCAObj()
                    ex.appendRun(midcaInst)

            curr_arson_chance -= ARSON_CHANCE_DECREMENT
        print()'''
        midcaInst = MIDCAInstance()
        midcaInst.createMIDCAObj()
        ex.appendRun(midcaInst,NUM_CYCLES)
        ex.run()


class MIDCAInstance():
    '''
    This class creates a specific instance of MIDCA given certain parameters.
    '''

    def __init__(self):


        self.initialized = False  # to initialize, call createMIDCAObj()
        self.myMidca = None

        self.world = None

    def createMIDCAObj(self):
        # in this demo, always keep extinguish to false
        extinguish = False

        thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
        GOAL_GRAPH_CMP_FUNC = minecraft_util.preferknows
        DISPLAY_FUNC = print
        MIDCA_ROOT = thisDir + "/../"

        DOMAIN_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/plant-protection-actual.pddl"
        EVENT_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/plant-protection-actual.pddl"
        STATE_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/plant_pro_actual.pddl"
        STATE_FILE_C = MIDCA_ROOT + "domains/ffdomain/minecraft/plant_pro_actual_2 (copy).pddl"
        DOMAIN_FILE_P = MIDCA_ROOT + "domains/ffdomain/minecraft/plant-protection-modif.pddl"
        EVENT_FILE_P = MIDCA_ROOT + "domains/ffdomain/minecraft/plant-protection-modif.pddl"
        STATE_FILE_P = MIDCA_ROOT + "domains/ffdomain/minecraft/plant-pro_modif.pddl"
        DATA_FILENAME = MIDCA_ROOT + "domains/ffdomain/minecraft/sas_plan"

        a = list(range(5))
        b = random.choice(a[1:])
        print(a, b)
        nat, inv = [], []

        for x in range(b):
            temp = [random.choice(a), random.choice(a)]
            while (temp in nat or temp in inv):
                temp = [random.choice(a), random.choice(a)]
            nat.append(temp)
            temp1 = [random.choice(a), random.choice(a)]
            while (temp1 in nat or temp1 in inv):
                temp1 = [random.choice(a), random.choice(a)]
            inv.append(temp1)

        state_file_2 = shutil.copyfile(STATE_FILE, STATE_FILE_C)
        print(nat, inv)

        inputfile = open(STATE_FILE_P, 'r').readlines()
        write_file = open(STATE_FILE_P, 'w')
        for line in inputfile:
            write_file.write(line)
            if '(and' in line:
                for i in range(b):
                    native_plant = "(native-at pos%d-%d)" % (nat[i][0], nat[i][1])
                    invasive_plant = "(invasive-at pos%d-%d)" % (inv[i][0], inv[i][1])
                    write_file.write(native_plant + "\n")
                    write_file.write(invasive_plant + "\n")
        write_file.close()

        inputfile = open(state_file_2, 'r').readlines()
        write_file = open(state_file_2, 'w')
        for line in inputfile:
            write_file.write(line)
            if ':init' in line:
                for i in range(b):
                    native_plant = "(native-at pos%d-%d)" % (nat[i][0], nat[i][1])
                    invasive_plant = "(invasive-at pos%d-%d)" % (inv[i][0], inv[i][1])
                    request_1 = "(requested human1 agent1 native-at pos%d-%d )" % (nat[i][0], nat[i][1])
                    request_2 = "(requested human1 agent1 not-invasive-at pos%d-%d )" % (inv[i][0], inv[i][1])
                    write_file.write(native_plant + "\n")
                    write_file.write(invasive_plant + "\n")
                    write_file.write(request_1 + "\n")
                    write_file.write(request_2 + "\n")
        write_file.close()

        world = pddlread.load_domain(DOMAIN_FILE, state_file_2, EVENT_FILE)
        world1 = pddlread.load_domain(DOMAIN_FILE_P, STATE_FILE_C, EVENT_FILE_P)


        # creates a PhaseManager object, which wraps a MIDCA object
        myMidca = base.PhaseManager(world, display='', verbose=4)
        # add phases by name
        for phase in ["Simulate", "Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
            myMidca.append_phase(phase)

        # add the modules which instantiate basic blocksworld operation
        myMidca.append_module("Simulate", simulator.MidcaActionSimulator())
        # myMidca.append_module("Simulate", simulator.MidcaEventSimulator())
        # myMidca.append_module("Simulate", simulator.ASCIIWorldViewer())
        # myMidca.append_module("Perceive", perceive.PerfectObserver())
        myMidca.append_module("Perceive", perceive.RemoteObserver())
        myMidca.append_module("Interpret", guide.UserGoalInput())
        # myMidca.append_module("Interpret", guide.MultipleGoalGen())
        myMidca.append_module("Interpret", note.GoalDiscrepancyDetector(world1))

        # myMidca.append_module("Interpret", note.GoalDiscrepancyDetector())
        myMidca.append_module("Eval", evaluate.SimpleEval())
        myMidca.append_module("Intend", intend.SimpleIntend())
        # myMidca.append_module("Plan", planning.MetricFFPlanner(
        # minecraft_util.ff_goals_from_midca_goals,
        # minecraft_util.ff_state_from_midca_world,
        # DOMAIN_FILE,
        # STATE_FILE))
        myMidca.append_module("Plan", planning.fastdownPlanner(
            minecraft_util.ff_goals_from_midca_goals,
            minecraft_util.ff_state_from_midca_world,
            DOMAIN_FILE_P,
            STATE_FILE_P, world1))
        myMidca.append_module("Act", act.SimpleAct())
        # myMidca.insert_module('Simulate', simulator.ArrowSimulator(arrowStart=1), 1)
        # myMidca.insert_module('Simulate', simulator.AttackSimulator(skeletonStart=2), 1)
        # myMidca.insert_module('Interpret', guide.TFFire(), 2)
        # myMidca.insert_module('Interpret', guide.Informhuman(), 3)

        # Set world viewer to output text
        myMidca.set_display_function(DISPLAY_FUNC)

        # tells the PhaseManager to copy and store MIDCA states so they can be accessed later.
        myMidca.storeHistory = True
        myMidca.initGoalGraph(cmpFunc=GOAL_GRAPH_CMP_FUNC)
        myMidca.init()
        #myMidca.run()

        '''def preferApprehend(goal1, goal2):
            if 'predicate' not in goal1 or 'predicate' not in goal2:
                return 0
            elif goal1['predicate'] == 'free' and goal2['predicate'] != 'free':
                return -1
            elif goal1['predicate'] != 'free' and goal2['predicate'] == 'free':
                return 1
            elif goal1['predicate'] == 'onfire' and goal2['predicate'] != 'onfire':
                return -1
            elif goal1['predicate'] != 'onfire' and goal2['predicate'] == 'onfire':
                return 1
            return 0'''

        # tells the PhaseManager to copy and store MIDCA states so they can be accessed later.
        myMidca.storeHistory = False
        #myMidca.initGoalGraph(cmpFunc=preferApprehend)
        ## DO NOT DO THIS: experiment.py will do this automatically: myMidca.init()

        '''print()
        "Created MIDCA " + str(id(myMidca)) + " w/ arsonchance=" + str(
            self.arsonChanceArg) + ", usingTFTreeFire=" + str(self.usingTFTreeFire) + ",usingSimMA=" + str(
            self.usingSimulatedMA)'''

        self.myMidca = myMidca
        self.initialized = True

    def run_cycles(self, num):
        for cycle in range(num):
            self.myMidca.one_cycle(verbose=0, pause=0)
            # self.myMidca.display(self.myMidca.midca.world)
            # print str(self.myMidca.midca.mem.get(self.myMidca.midca.mem.GOAL_GRAPH))
            # print str(cycle)

    def getMIDCAObj(self):
        return self.myMidca

    def __str__(self):
        s = "MIDCAInstance [id]="
        '''s += "\n[arsonchance]=" + str(self.arsonChanceArg)
        s += "\n[usingTFTreeFire]=" + str(self.usingTFTreeFire)
        s += "\n[usingSimMA]=" + str(self.usingSimulatedMA)
        s += "\n[Score]=" + str(self.myMidca.mem.get(evaluate.SCORE))'''
        return s


if __name__ == "__main__":
    ArsonCogSciDemo()
