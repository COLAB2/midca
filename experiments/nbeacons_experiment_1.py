'''
Created on Jul 16, 2015

This file runs an experiment involving multiple, differently configured, instances of MIDCA
and collecting data from MIDCA's memory and the experiment.

@author: Dustin Dannenhauer
'''
from MIDCA import base
from MIDCA.worldsim import domainread, stateread
from MIDCA.modules import simulator, perceive, note, guide, evaluate, intend, planning, act
from MIDCA.domains import nbeacons
from MIDCA.domains.nbeacons import nbeacons_util
from MIDCA.domains.nbeacons.plan import methods_nbeacons, operators_nbeacons
import datetime
import os
import inspect
import time
from multiprocessing import Pool
import sys
import ctypes # for popups

DATADIR = "experiments/nbeacons-experiment-1-data/"
NOW_STR = datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d--%H-%M-%S')
DATA_FILENAME = DATADIR + "NBeaconsExperiment1" + NOW_STR + ".csv"
DATA_FILE_HEADER_STR = "runID,beaconFailRate,windDir,goalsAchieved,actionsExecuted,planningCounts,numCycles\n"

NUM_CYCLES = 1000

NUM_PROCESSES = 8 # Number of individual python processes to use

def singlerun_output_str(run_id, curr_midca, beaconFailRate, windDir, num_cycles):
    # num times planning from scratch
    planning_count = curr_midca.mem.get(curr_midca.mem.PLANNING_COUNT)
    goals_achieved = curr_midca.mem.get(curr_midca.mem.GOALS_ACHIEVED)
    actions_executed = curr_midca.mem.get(curr_midca.mem.ACTIONS_EXECUTED)

    result = str(run_id) + "," + str(beaconFailRate) + "," + str(windDir) + "," + str(goals_achieved) + "," + str(actions_executed) + "," + str(planning_count)+","+str(NUM_CYCLES) + "\n"
    return result
    
def singlerun(args):
    run_id = args[0]
    beaconFailRate = args[1]
    windDir = args[2]
    midca_inst = MIDCAInstance(beaconFailRate=beaconFailRate,windDir=windDir)
    midca_inst.createMIDCAObj()
    curr_midca = midca_inst.getMIDCAObj()
    curr_midca.init()
    midca_inst.run_cycles(NUM_CYCLES)
    
    # prepare data for writing output string
    result_str = singlerun_output_str(run_id,curr_midca,beaconFailRate, windDir, NUM_CYCLES)
    return result_str 

def runexperiment():  
    runs = []
    # args are [runID, beaconFailRate, windDir]
    no_beacon_fail_no_wind_args = [0,0,'off']
    beacon_fail_no_wind_args = [1,3,'off']
    beacon_fail_and_wind_args = [2,3,'east']
    
    runs.append(no_beacon_fail_no_wind_args)
    runs.append(beacon_fail_no_wind_args)
    runs.append(beacon_fail_and_wind_args)
            
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

class MIDCAInstance():
    '''
    This class creates a specific instance of MIDCA given certain parameters.
    '''
    def __init__(self, beaconFailRate, windDir):
        self.beaconFailRate = beaconFailRate
        self.windDir = windDir
        self.initialized = False # to initialize, call createMIDCAObj()
        self.myMidca = None
        self.world = None

    def createMIDCAObj(self):
        # Setup
        thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
        MIDCA_ROOT = thisDir + "/../"
        
        ### Domain Specific Variables
        DOMAIN_ROOT = MIDCA_ROOT + "domains/nbeacons/"
        DOMAIN_FILE = DOMAIN_ROOT + "domains/nbeacons.sim"
        #STATE_FILE = DOMAIN_ROOT + "states/.sim" # state file is generated dynamically
        DISPLAY_FUNC = nbeacons_util.drawNBeaconsScene
        DECLARE_METHODS_FUNC = methods_nbeacons.declare_methods
        DECLARE_OPERATORS_FUNC = operators_nbeacons.declare_operators
        GOAL_GRAPH_CMP_FUNC = None
        DIMENSION = 10
        
        # percent chance each beacon will fail each tick
        BEACON_FAIL_RATE = self.beaconFailRate
        
        # Load domain
        world = domainread.load_domain(DOMAIN_FILE)
        
        # Create Starting state
        state1 = nbeacons_util.NBeaconGrid()
        state1.generate(width=DIMENSION,height=DIMENSION)
        state1_str = state1.get_STRIPS_str()
        
        # Load state
        stateread.apply_state_str(world, state1_str)
        
        # Creates a PhaseManager object, which wraps a MIDCA object
        myMidca = base.PhaseManager(world, display=DISPLAY_FUNC, verbose=2)
        
        # Add phases by name
        for phase in ["Simulate", "Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
            myMidca.append_phase(phase)
        
        # Add the modules which instantiate basic operation
        #myMidca.append_module("Simulate", simulator.MidcaActionSimulator())
        myMidca.append_module("Simulate", simulator.NBeaconsActionSimulator(wind=True,wind_dir=self.windDir,dim=DIMENSION))
        myMidca.append_module("Simulate", simulator.NBeaconsSimulator(beacon_fail_rate=BEACON_FAIL_RATE))
        myMidca.append_module("Simulate", simulator.ASCIIWorldViewer(DISPLAY_FUNC))
        myMidca.append_module("Perceive", perceive.PerfectObserver())
        
        #myMidca.append_module("Interpret", guide.UserGoalInput())
        myMidca.append_module("Interpret", guide.RandomActivationGoals())
        myMidca.append_module("Eval", evaluate.NBeaconsDataRecorder())
        myMidca.append_module("Intend", intend.SimpleIntend())
        myMidca.append_module("Plan", planning.PyHopPlanner(nbeacons_util.pyhop_state_from_world,
                                                            nbeacons_util.pyhop_tasks_from_goals,
                                                            DECLARE_METHODS_FUNC,
                                                            DECLARE_OPERATORS_FUNC)) # set up planner for sample domain
        myMidca.append_module("Act", act.SimpleAct())
        
        # Set world viewer to output text
        myMidca.set_display_function(nbeacons_util.drawNBeaconsScene) 
        
        # Tells the PhaseManager to copy and store MIDCA states so they can be accessed later.
        # Note: Turning this on drastically increases MIDCA's running time.
        myMidca.storeHistory = False
        myMidca.mem.logEachAccess = False
        
        # Initialize and start running!
        # myMidca.init() # not called here, called in MidcaRun
        myMidca.initGoalGraph()
        #myMidca.run()

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

def bargraph(prev_file):
    from mpl_toolkits.mplot3d import Axes3D
    import matplotlib.pyplot as plt
    from matplotlib import cm
    import numpy as np
    
    # get the most recent filename
    files = sorted([f for f in os.listdir(DATADIR)])
    datafile = DATADIR + files[-(prev_file+1)]
    print("-- About to graph data from "+str(datafile))
    header = True
    goals_achieved = []
    actions_executed = []
    planning_counts = []
    
    with open(datafile,'r') as f:
        for line in f.readlines():
            if header: 
                header = False
            else:
                row = line.strip().split(',')
                goals_achieved.append(int(row[3]))
                actions_executed.append(int(row[4]))
                planning_counts.append(int(row[5]))
                
    agents_N = 3
    width = 0.3
    ind = np.arange(agents_N)
    
    fig, ax = plt.subplots()
    rects1 = ax.bar(ind, goals_achieved, width, color='r',label="Goals Achieved")
    rects2 = ax.bar(ind+width, actions_executed, width, color='b',label="Actions Executed")
    rects3 = ax.bar(ind+(2*width), planning_counts, width, color='g',label="# Replanning")
    ax.set_xticks(ind + width)
    ax.set_xticklabels(("0% BF No Wind","3% BF No Wind","3% BF Yes Wind"))
    ax.legend((rects1[0], rects2[0], rects3[0]), ('Goals Achieved', 'Actions Executed', "# Replanning"))
    plt.legend(loc=2)
    plt.show()

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
            bargraph(int(sys.argv[2]))
        else:
            bargraph(0)
    elif len(sys.argv) > 1 and sys.argv[1] == 'graphslices':
        graph_slices_hardcoded()
    else:   
        runexperiment()
          