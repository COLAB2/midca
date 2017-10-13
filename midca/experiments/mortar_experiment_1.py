'''
Created on Jul 16, 2015

This file runs an experiment involving multiple, differently configured, instances of MIDCA
and collecting data from MIDCA's memory and the experiment.

@author: Dustin Dannenhauer
'''
from midca import base
from midca.worldsim import domainread, stateread, blockstate
from midca.modules import simulator, perceive, note, guide, evaluate, intend, planning, act
import datetime
import os
import inspect
import time
from multiprocessing import Pool
import sys
import ctypes # for popups

DATADIR = "experiments/mortar-experiment-1-data/"
NOW_STR = datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d--%H-%M-%S')
DATA_FILENAME = DATADIR + "MortarCogSciDemoExperiment1" + NOW_STR + ".csv"
DATA_FILE_HEADER_STR = "runID,numMortar,towersCompleted,score,numcycles\n"

CYCLES_START = 10
CYCLES_END = 50
CYCLES_INCREMENT = 10

MORTAR_QUANTITY_START = 0
MORTAR_QUANTITY_END = 10
MORTAR_QUANTITY_INCREMENT = 1 # this should ideally be a function

NUM_PROCESSES = 8 # Number of individual python processes to use

def singlerun_output_str(run_id, curr_midca, curr_mortar_count, num_cycles):
    towersCompleted = curr_midca.mem.get(evaluate.MORTARSCORE).getTowersCompleted()
    towersScore = curr_midca.mem.get(evaluate.MORTARSCORE).getTowersScore()
    numMortars = curr_mortar_count
    result = str(run_id) + "," + str(numMortars) + "," + str(towersCompleted) + "," + str(towersScore) + "," + str(num_cycles) + "\n"
    return result
    
def singlerun(args):
    run_id = args[0]
    curr_mortar_count = args[1]
    midca_inst = MIDCAInstance(curr_mortar_count)
    midca_inst.createMIDCAObj()
    num_cycles = args[2]
    curr_midca = midca_inst.getMIDCAObj()
    curr_midca.init()
    midca_inst.run_cycles(num_cycles)
    
    # prepare data for writing output string
    result_str = singlerun_output_str(run_id,curr_midca,curr_mortar_count, num_cycles)
    return result_str 

def runexperiment():  
    runs = []
    curr_mortar_count = MORTAR_QUANTITY_START
    run_id = 0
    while curr_mortar_count <= MORTAR_QUANTITY_END:
        curr_cycles_count = CYCLES_START
        while curr_cycles_count <= CYCLES_END:
            curr_args = [run_id,curr_mortar_count, curr_cycles_count]
            runs.append(curr_args)
            run_id+=1
            curr_cycles_count+= CYCLES_INCREMENT
        curr_mortar_count += MORTAR_QUANTITY_INCREMENT
            
    # Uses multiprocessing to give each run its own python process
    print("-- Starting experiment using "+str(NUM_PROCESSES)+" processes...")
    t0 = time.time()
    # **** NOTE: it is very important chunksize is 1 and maxtasksperchild is 1
    # **** (each MIDCA must use its own python process)
    pool = Pool(processes=NUM_PROCESSES, maxtasksperchild=1)
    results = pool.map(singlerun, runs, chunksize=1)
    t1 = time.time()
    timestr = '%.2f' % (t1-t0)
    print("-- Experiment finished! Took "+timestr+"s, generated "+str(len(results))+" data points")
    print("-- Writing data to file...")
    f = open(DATA_FILENAME, 'w')
    f.write(DATA_FILE_HEADER_STR)
    for r in results:
        f.write(r)
    print("-- Data written to file "+str(DATA_FILENAME))
    print("-- Experiment complete!")

def asqiiDisplay(world):
    '''
    Creates an asqii representation for blocksworld.
    '''
    blocks = blockstate.get_block_list(world)
    #print str(scene.Scene(blocks))

class MIDCAInstance():
    '''
    This class creates a specific instance of MIDCA given certain parameters.
    '''
    def __init__(self, currMortarCount):
        self.currMortarCount = currMortarCount
        self.initialized = False # to initialize, call createMIDCAObj()
        self.myMidca = None
        self.world = None

    def createMIDCAObj(self):
        extinguish = False
        mortar = True

        thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

        MIDCA_ROOT = thisDir + "/../"

        domainFile = MIDCA_ROOT + "worldsim/domains/arsonist_mortar.sim"
        stateFile = MIDCA_ROOT + "worldsim/states/defstate_mortar.sim"

        # load domain file like normal
        self.world = domainread.load_domain(domainFile)
        
        # for state file, need to add number of mortar blocks to begin with
        state_str = open(stateFile).read() # first read file
        # now add new mortar blocks
        for i in range(self.currMortarCount):
            state_str+="MORTARBLOCK(M"+str(i)+")\n"
            state_str+="available(M"+str(i)+")\n"
        # now load the state    
        stateread.apply_state_str(self.world, state_str)
        # creates a PhaseManager object, which wraps a MIDCA object
        myMidca = base.PhaseManager(self.world, display=asqiiDisplay,verbose=0)
        #asqiiDisplay(world)
        # add phases by name
        for phase in ["Simulate", "Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
            myMidca.append_phase(phase)

        # add the modules which instantiate basic blocksworld operation
        myMidca.append_module("Simulate", simulator.MidcaActionSimulator())
        myMidca.append_module("Simulate", simulator.ASCIIWorldViewer())
        myMidca.append_module("Perceive", perceive.PerfectObserver())
        myMidca.append_module("Interpret", note.ADistanceAnomalyNoter())
        # need to make sure to disable all user input modules #myMidca.append_module("Interpret", guide.UserGoalInput())
        myMidca.append_module("Eval", evaluate.SimpleEval())
        myMidca.append_module("Intend", intend.SimpleIntend())
        myMidca.append_module("Plan", planning.PyHopPlanner(extinguish, mortar))
        myMidca.append_module("Act", act.SimpleAct())

        #myMidca.insert_module('Simulate', simulator.ArsonSimulator(arsonChance=self.arsonChanceArg, arsonStart=10), 1)
        #myMidca.insert_module('Simulate', simulator.FireReset(), 0)
        myMidca.insert_module('Interpret', guide.TFStack(), 1)

        myMidca.insert_module('Eval', evaluate.MortarScorer(), 1)  # this needs to be a 1 so that Scorer happens AFTER SimpleEval
        # tells the PhaseManager to copy and store MIDCA states so they can be accessed later.
        myMidca.storeHistory = False
        myMidca.initGoalGraph()
        ## note: myMidca.init() is NOT called here, instead in singlerun()

        self.myMidca = myMidca
        self.initialized = True

    def run_cycles(self, num):
        for cycle in range(num):
            self.myMidca.one_cycle(verbose = 0, pause = 0)
            
    def getMIDCAObj(self):
        return self.myMidca

    def __str__(self):
        if self.myMidca:
            s = "MIDCAInstance [id]="+str(id(self.myMidca))
            s += "\n[currMortarCount]="+str(self.currMortarCount)
            s += "\n[Score]="+str(self.myMidca.mem.get(evaluate.MORTARSCORE))
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
    max_scores = {10:6,20:10,30:20,40:26,50:30,60:40,70:46,80:50,90:60,100:66}
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
    datafile = DATADIR + files[-(prev_file+1)]
    print("-- About to graph data from "+str(datafile))
    header = True
    mortar_ys = []
    cycles_xs = []
    score_zs = []
    
    with open(datafile,'r') as f:
        for line in f.readlines():
            if header: 
                header = False
            else:
                row = line.strip().split(',')
                mortar_ys.append(int(row[1]))
                num_cycles = int(row[4])
                score = int(row[3])
                score = (score*1.0) / get_max_score_for_cycles(num_cycles)
                score_zs.append(score)
                cycles_xs.append(num_cycles)
                
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_trisurf(cycles_xs, mortar_ys,score_zs,cmap=cm.coolwarm)
    ax.set_zlim(bottom=0.0,top=1.0)
    ax.set_xlim(max(cycles_xs),0)
    ax.set_ylim(max(mortar_ys),0)
    ax.legend()
    ax.set_xlabel("Goals")
    ax.set_ylabel("Resources")
    ax.set_zlabel("Score")
    plt.show()

def graph_slices_hardcoded():
    '''
    Produce the graph
    '''
    # as soon as I generate new data, I need to update these numbers: 3 and 4
    # they correspond to the data files sorted by most recent first
    prev_file_goal_trans = 6
    prev_file_no_goal_trans = 5
    
    from mpl_toolkits.mplot3d import Axes3D
    import matplotlib.pyplot as plt
    from matplotlib import cm
    # get the most recent filename
    files = sorted([f for f in os.listdir(DATADIR)])
    datafile_goal_trans = DATADIR + files[-(prev_file_goal_trans+1)]
    datafile_no_goal_trans = DATADIR + files[-(prev_file_no_goal_trans+1)]
    print("-- About to read in goal transform data from "+str(datafile_goal_trans))
    header = True
    gt_mortar_ys = []
    gt_cycles_xs = []
    gt_score_zs = []
    count = 0
    with open(datafile_goal_trans,'r') as f:
        for line in f.readlines():
            if header: 
                header = False
            else:
                count+=1
                row = line.strip().split(',')
                gt_mortar_ys.append(int(row[1]))
                num_cycles = int(row[4])
                score = int(row[3])
                score = (score*1.0) / get_max_score_for_cycles(num_cycles)
                gt_score_zs.append(score)
                gt_cycles_xs.append(num_cycles)
        print("There were "+str(count)+" data points collected that will be used for this graph")
    print("-- About to read in non-goal transform data from "+str(datafile_goal_trans))
    
    no_gt_mortar_ys = []
    no_gt_cycles_xs = []
    no_gt_score_zs = []
    header = True
    with open(datafile_no_goal_trans,'r') as f:
        for line in f.readlines():
            if header: 
                header = False
            else:
                row = line.strip().split(',')
                no_gt_mortar_ys.append(int(row[1]))
                num_cycles = int(row[4])
                score = int(row[3])
                score = (score*1.0) / get_max_score_for_cycles(num_cycles)
                no_gt_score_zs.append(score)
                no_gt_cycles_xs.append(num_cycles)
    
    # hold mortar at 15
    mortar_hold = 5
    
    # now get all data points where mortar is the hold value
    gt_score = []
    gt_cycles = [] 

    for i in range(len(gt_mortar_ys)):
        curr_mortar = gt_mortar_ys[i]
        curr_score = gt_score_zs[i]
        curr_cycles = gt_cycles_xs[i]
        if curr_mortar == mortar_hold:
            gt_score.append(curr_score)
            gt_cycles.append(curr_cycles)

    no_gt_score = []
    no_gt_cycles = []
    
    for i in range(len(no_gt_mortar_ys)):
        curr_mortar = no_gt_mortar_ys[i]
        curr_score = no_gt_score_zs[i]
        curr_cycles = no_gt_cycles_xs[i]
        if curr_mortar == mortar_hold:
            no_gt_score.append(curr_score)
            no_gt_cycles.append(curr_cycles)
    
    # now graph slice where x-axis is number of goals
    
    plt.plot(gt_cycles,gt_score,label='Goal Trans', linewidth=3)
    plt.plot(no_gt_cycles,no_gt_score,'--',label='No Goal Trans',linewidth=3)
    #ax.plot_trisurf(cycles_xs, mortar_ys,score_zs,cmap=cm.coolwarm)
    #ax.set_zlim(bottom=0.0,top=1.0)
    #ax.set_xlim(max(cycles_xs),0)
    #ax.set_ylim(max(mortar_ys),0)
    plt.legend()
    plt.xlabel("Goals in Mortar Towers to Build")
    plt.ylabel("Score")
    plt.rcParams.update({'font.size': 16})
    #fig.set_zlabel("Score")
    plt.show()

    # now do the exact same thing, except hold goals at 
    cycles_hold = 100

    # now get all data points where mortar is the hold value
    gt_score = []
    gt_mortar = [] 

    for i in range(len(gt_mortar_ys)):
        curr_mortar = gt_mortar_ys[i]
        curr_score = gt_score_zs[i]
        curr_cycles = gt_cycles_xs[i]
        if curr_cycles == cycles_hold:
            gt_score.append(curr_score)
            gt_mortar.append(curr_mortar)

    no_gt_score = []
    no_gt_mortar = []
    
    for i in range(len(no_gt_mortar_ys)):
        curr_mortar = no_gt_mortar_ys[i]
        curr_score = no_gt_score_zs[i]
        curr_cycles = no_gt_cycles_xs[i]
        if curr_cycles == cycles_hold:
            no_gt_score.append(curr_score)
            no_gt_mortar.append(curr_mortar)
    
    # now graph slice where x-axis is number of goals
    
    plt.plot(gt_mortar,gt_score,label='Goal Trans', linewidth=3)
    plt.plot(no_gt_mortar,no_gt_score,'--',label='No Goal Trans',linewidth=3)
    #ax.plot_trisurf(cycles_xs, mortar_ys,score_zs,cmap=cm.coolwarm)
    #ax.set_zlim(bottom=0.0,top=1.0)
    #ax.set_xlim(max(cycles_xs),0)
    #ax.set_ylim(max(mortar_ys),0)
    plt.legend(loc=4)
    plt.xlabel("Resources in Number of Mortar")
    plt.ylabel("Score")
    #fig.set_zlabel("Score")
    plt.rcParams.update({'font.size': 16})
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == 'graph':
        # produce graph instead of running experiment
        if len(sys.argv) > 2:
            graph(int(sys.argv[2]))
        else:
            graph(0)
    elif len(sys.argv) > 1 and sys.argv[1] == 'graphslices':
        graph_slices_hardcoded()
    else:   
        runexperiment()
          