'''
Created on Jul 16, 2015

This file runs an experiment involving multiple, differently configured, instances of MIDCA
and collecting data from MIDCA's memory and the experiment.

@author: Dustin Dannenhauer
'''
#!/usr/bin/env python
from midca import base
from midca.modules import simulator, guide, evaluate, perceive, note, intend, planning, act
from midca.worldsim import stateread
from midca.worldsim import pddl_num_read as pddlread
from midca.worldsim import problem_generator
import inspect, os
from shutil import copyfile

# Domain Specific Imports
from midca.domains.ffdomain.minecraft import minecraft_util
import inspect
import time
from multiprocessing import Pool
import sys
import ctypes  # for popups
GOAL_GRAPH_CMP_FUNC = minecraft_util.preferSurvive

DATADIR = "experiments/mortar-experiment-1-data/"
# NOW_STR = datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d--%H-%M-%S')
# DATA_FILENAME = DATADIR + "MortarCogSciDemoExperiment1" + NOW_STR + ".csv"
DATA_FILE_HEADER_STR = "runID,numMortar,towersCompleted,score,numcycles\n"

CYCLES_START = 10
CYCLES_END = 50
CYCLES_INCREMENT = 10

MORTAR_QUANTITY_START = 0
MORTAR_QUANTITY_END = 10
MORTAR_QUANTITY_INCREMENT = 1  # this should ideally be a function

NUM_PROCESSES = 8  # Number of individual python processes to use


def singlerun_output_str(run_id, curr_midca, num_cycles):
    health = curr_midca.mem.get(curr_midca.mem.AGENT_HEALTH)
    print(health)
    tree = curr_midca.mem.get(curr_midca.mem.TREE_HARVEST)
    print(tree)
    dead_point = curr_midca.mem.get(curr_midca.mem.AGENT_DEAD_CYCLE)
    print(dead_point)
    midca_cycle = curr_midca.mem.get(curr_midca.mem. MIDCA_CYCLES)
    num_actions = curr_midca.mem.get(curr_midca.mem.ACTIONS_EXECUTED)
    result = str(dead_point)+"," + str(num_actions) + "," +str(run_id) + "," + \
             str(health) + "," + str(tree) + "," + str(
        num_cycles) + "\n"
    return result

def stop_point(curr_midca):
    alive = curr_midca.mem.get(curr_midca.mem.AGENT_ALIEVE)
    if not alive:
        return True

def singlerun(args):
    run_id = 0

    midca_inst = MIDCAInstance()
    midca_inst.createMIDCAObj()
    num_cycles = 50
    curr_midca = midca_inst.getMIDCAObj()
    curr_midca.init()
    midca_inst.run_cycles(num_cycles)

    # prepare data for writing output string
    result_str = singlerun_output_str(run_id, curr_midca, num_cycles)
    return result_str


def runexperiment():
    runs = []
    thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

    MIDCA_ROOT = thisDir + "/../"

    STATE_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/wood"
    run_id = 0
    for i in range(10):
        run_id = i
        problem_generator.generate_file(STATE_FILE + str(i))
        copyfile(STATE_FILE + str(i), STATE_FILE + str(i) + "_copy")
        results = singlerun(run_id)

    # Uses multiprocessing to give each run its own python process
    print("-- Starting experiment using")
    t0 = time.time()
    # **** NOTE: it is very important chunksize is 1 and maxtasksperchild is 1
    # **** (each MIDCA must use its own python process)


    t1 = time.time()
    timestr = '%.2f' % (t1 - t0)
    print(("-- Experiment finished! Took " + timestr + "s, generated " + str(len(results)) + " data points"))
    print("-- Writing data to file...")

    print(results)

    print("-- Experiment complete!")





class MIDCAInstance():
    '''
    This class creates a specific instance of MIDCA given certain parameters.
    '''

    def __init__(self):

        self.initialized = False  # to initialize, call createMIDCAObj()
        self.myMidca = None
        self.world = None

    def createMIDCAObj(self):
        thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

        MIDCA_ROOT = thisDir + "/../"

        DOMAIN_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/domain.pddl"
        EVENT_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/domain_1.pddl"
        STATE_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/wood"

        world = pddlread.load_domain(DOMAIN_FILE, STATE_FILE, EVENT_FILE)
        myMidca = base.PhaseManager(world, display='', verbose=4)
        # add phases by name
        for phase in ["Simulate", "Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
            myMidca.append_phase(phase)

        # add the modules which instantiate basic blocksworld operation
        myMidca.append_module("Simulate", simulator.MidcaActionSimulator())
        myMidca.append_module("Simulate", simulator.MidcaEventSimulator())
        # myMidca.append_module("Simulate", simulator.ASCIIWorldViewer(display=DISPLAY_FUNC))
        myMidca.append_module("Perceive", perceive.PerfectObserver())
        myMidca.append_module("Interpret", guide.SimpleMinecraftGoalGen())
        myMidca.append_module("Eval", evaluate.SimpleEvalSubgoals())
        myMidca.append_module("Intend", intend.SimpleIntendwithSubgoals())
        myMidca.append_module("Plan", planning.MetricFFPlanner(
            minecraft_util.ff_goals_from_midca_goals,
            minecraft_util.ff_state_from_midca_world,
            DOMAIN_FILE,
            STATE_FILE
        ))
        myMidca.append_module("Act", act.SimpleAct())
        myMidca.insert_module('Interpret', guide.ReactiveSurvive(), 3)

        myMidca.storeHistory = True
        myMidca.initGoalGraph(cmpFunc=GOAL_GRAPH_CMP_FUNC)

        self.myMidca = myMidca
        self.initialized = True

    def run_cycles(self, num):
        for cycle in range(num):
            self.myMidca.one_cycle(verbose=2, pause=0)

    def getMIDCAObj(self):
        return self.myMidca

    def __str__(self):
        if self.myMidca:
            s = "MIDCAInstance [id]=" + str(id(self.myMidca))

            s += "\n[Score]=" + str(self.myMidca.mem.get(self.myMidca.mem.TREE_HARVEST))
            s += "\n[Score]=" + str(self.myMidca.mem.get(self.myMidca.mem.AGENT_HEALTH))

            return s
        else:
            return 'not-initialized'


###########
## Graph ##
###########

def get_max_score_for_cycles(cycle):
    '''
    Used to convert score into percent. These hardcoded scores are the max scores for the corresponding cycles.
    '''
    max_scores = {10: 6, 20: 10, 30: 20, 40: 26, 50: 30, 60: 40, 70: 46, 80: 50, 90: 60, 100: 66}
    return max_scores[cycle]


def graph(prev_file):
    '''
    Produce the graph
    '''
    from mpl_toolkits.mplot3d import Axes3D
    import matplotlib.pyplot as plt
    from matplotlib import cm
    # get the most recent filename
    files = sorted([f for f in os.listdir(DATADIR)])
    datafile = DATADIR + files[-(prev_file + 1)]
    print(("-- About to graph data from " + str(datafile)))
    header = True
    mortar_ys = []
    cycles_xs = []
    score_zs = []

    with open(datafile, 'r') as f:
        for line in f.readlines():
            if header:
                header = False
            else:
                row = line.strip().split(',')
                mortar_ys.append(int(row[1]))
                num_cycles = int(row[4])
                score = int(row[3])
                score = (score * 1.0) / get_max_score_for_cycles(num_cycles)
                score_zs.append(score)
                cycles_xs.append(num_cycles)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_trisurf(cycles_xs, mortar_ys, score_zs, cmap=cm.coolwarm)
    ax.set_zlim(bottom=0.0, top=1.0)
    ax.set_xlim(max(cycles_xs), 0)
    ax.set_ylim(max(mortar_ys), 0)
    ax.legend()
    ax.set_xlabel("Goals")
    ax.set_ylabel("Resources")
    ax.set_zlabel("Score")
    plt.show()





if __name__ == "__main__":
    # if len(sys.argv) > 1 and sys.argv[1] == 'graph':
    #     # produce graph instead of running experiment
    #     if len(sys.argv) > 2:
    #         graph(int(sys.argv[2]))
    #     else:
    #         graph(0)
    # elif len(sys.argv) > 1 and sys.argv[1] == 'graphslices':
    #     graph_slices_hardcoded()
    # else:
    thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

    MIDCA_ROOT = thisDir + "/../"

    STATE_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/wood.1.pddl"
    # problem_generator.generate_file(STATE_FILE)

    runexperiment()
