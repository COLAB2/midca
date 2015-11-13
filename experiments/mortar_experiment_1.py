'''
Created on Jul 16, 2015

@author: dustin
'''
#from MIDCA.experiment.experiment import Experiment
from MIDCA import base
from MIDCA.worldsim import domainread, stateread, blockstate, scene
from MIDCA.modules import simulator, perceive, note, guide, evaluate, intend, planning, act
import datetime
import os
import inspect
import time
import copy
from functools import partial
from multiprocessing import Pool
import sys

DATADIR = "experiments/mortar-experiment-1-data/"

def asqiiDisplay(world):
    '''
    Creates an asqii representation for blocksworld.
    '''
    blocks = blockstate.get_block_list(world)
    #print str(scene.Scene(blocks))



##############################################        
        # func for Pool.map() (there are some restrictions on the kinds of functions it can take)
def singlerun(args):
    run_id = args[0]
    curr_mortar_count = args[1]
    midca_inst = MIDCAInstance(curr_mortar_count)
    midca_inst.createMIDCAObj()
    num_cycles = args[2]
    curr_midca = midca_inst.getMIDCAObj()
    curr_midca.init()
    ##print("****************************** reached post init() *****************")
    midca_inst.run_cycles(num_cycles)
    ##print("****************************** reached post run_cycles() *****************")
    
    
    
    # prepare data for writing output string
    towersCompleted = curr_midca.mem.get(evaluate.MORTARSCORE).getTowersCompleted()
    towersScore = curr_midca.mem.get(evaluate.MORTARSCORE).getTowersScore()
    numMortars = curr_midca.mem.get(evaluate.MORTARSCORE).getMortarBlocks()
    result = str(run_id) + "," + str(numMortars) + "," + str(towersCompleted) + "," + str(towersScore) + "," + str(num_cycles) + "\n"
    print(result) 
    return result

class MortarCogSciDemoExperiment1():
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

        def customWriteData(run_id, num_cycles, midca):
            '''
            This function will be called on each midca object after it has run and should get the
            necessary values (to be written to a data file) from the midca object.

            Customize this function to retrieve the data you wish to collect during the experiment
            '''
            
            # get tower score
            towersCompleted = midca.mem.get(evaluate.MORTARSCORE).getTowersCompleted()
            towersScore = midca.mem.get(evaluate.MORTARSCORE).getTowersScore()
            #print("writing to file "+str(os.path.abspath(file.name))+": "+str(midca.mem.get(evaluate.MORTARSCORE)))
            numMortars = midca.mem.get(evaluate.MORTARSCORE).getMortarBlocks()
            return str(run_id) + "," + str(numMortars) + "," + str(towersCompleted) + "," + str(towersScore) +","+str(num_cycles+"\n")
            #file.write(str(run_id) + "," + str(numMortars) + "," + str(towersCompleted) + "," + str(towersScore) +","+str(num_cycles+"\n"))
            
            #file.flush()

        def cleanup():
            '''
            This function is used to execute any code that needs to be run
            before closing, (i.e. closing file handles)
            '''
            file.close()
        
        #NUM_CYCLES = 50

        ###### create code for each MIDCA run #####
        #ex = Experiment(self.__class__.__name__)

        #ex.addWriteDataFunc(customWriteData)

        CYCLES_START = 30
        CYCLES_END = 50
        CYCLES_INCREMENT = 1

        MORTAR_QUANTITY_START = 1
        MORTAR_QUANTITY_END = 10
        MORTAR_QUANTITY_INCREMENT = 1 # this should ideally be a function
        runs = []
        curr_mortar_count = MORTAR_QUANTITY_START
        run_id = 0
        while curr_mortar_count <= MORTAR_QUANTITY_END:
            curr_cycles_count = CYCLES_START
            while curr_cycles_count <= CYCLES_END:
                # create MIDCA instance
                #midcaInst = MIDCAInstance(curr_mortar_count)
                #midcaInst.createMIDCAObj()
                ##print("appending the run w/ mortarcount of " + str(curr_mortar_count))
                runs.append([run_id,curr_mortar_count, curr_cycles_count])
                run_id+=1
                curr_cycles_count+= CYCLES_INCREMENT
            curr_mortar_count += MORTAR_QUANTITY_INCREMENT
        #print "******************* Finished Initialization of runs array **************************"
        
        

        
        # Uses multiprocessing to give each run its own python process
        
        pool = Pool(processes=8, maxtasksperchild=1)
        # NOTE: it is very important chunksize is 1 (each MIDCA must use its own python process)
        results = pool.map(singlerun, runs, chunksize=1)
        print("Experiment finished. We've obtained "+str(len(results))+" data points")
        #return results
         
         
         
         
        ###############################################
        time.sleep(1)
        # Write data to file
        curr_datetime_str = datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d--%H-%M-%S')
        DATA_FILENAME = DATADIR + "MortarCogSciDemoExperiment1" + curr_datetime_str + ".csv"
        print("FILENAME = "+str(DATA_FILENAME))
        for r in results:
            print("  "+str(r))
        #DATA_FILENAME = DATADIR + filename
        # initialize the csv file here
        f = open(DATA_FILENAME, 'w')
        f.write("runID,numMortar,towersCompleted,score,numcycles\n")
        for r in results:
            f.write(r)
        
        # since we are varying using 3 parameters (arson chance, using tf trees, using MA)
        # we have three nested loops, creating individual midca runs for each unique paramterization

        # time.sleep(0.5)
        # 1. vary by arson chance
#         curr_mortar_count = MORTAR_QUANTITY_START
#         while curr_mortar_count < MORTAR_QUANTITY_END:
#             # create MIDCA instance
#             midcaInst = MIDCAInstance(curr_mortar_count)
#             midcaInst.createMIDCAObj()
#             #print("appending the run w/ mortarcount of "+str(curr_mortar_count))
#             ex.appendRun(midcaInst)
#             
#             curr_mortar_count += MORTAR_QUANTITY_INCREMENT
#         #print "*******************FINISHED**************************"
        # time.sleep(0.5)
        #ex.run()
        
        


class MIDCAInstance():
    '''
    This class creates a specific instance of MIDCA given certain parameters.
    '''

    def __init__(self, currMortarCount):
        self.currMortarCount = currMortarCount
        self.initialized = False # to initialize, call createMIDCAObj()
        self.myMidca = None

        self.world = None
        
#         globalsfile = 'experiments/mortar-experiment-1-data/globals-for-mortar-'+str(currMortarCount)+'.txt' 
#         with open(globalsfile, 'w') as f:
#             f.write('------------------ dir() ----------------------------------\n')
#             for name in dir():
#                 myvalue = eval(name)
#                 f.write(str(name)+ " is "+ str(type(name))+ " = "+ str(myvalue)+"\n")
#             f.write('------------------ locals() ----------------------------------\n')
#             for name in locals():
#                 myvalue = eval(name)
#                 f.write(str(name)+ " is "+ str(type(name))+ " = "+ str(myvalue)+"\n")
#             f.write('------------------ globals() ----------------------------------\n')
#             for name in globals():
#                 myvalue = eval(name)
#                 f.write(str(name)+ " is "+ str(type(name))+ " = "+ str(myvalue)+"\n")
#             f.write('------------------ vars() ----------------------------------\n')
#             for key,val in vars().items():
#                 f.write(str(key)+ " is "+ str(type(val))+ " = "+ str(val)+"\n")

    def createMIDCAObj(self):
        # in this demo, always keep extinguish to false
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
            myMidca = base.PhaseManager(self.world, display=asqiiDisplay,verbose=4)
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
            ## DO NOT DO THIS: experiment.py will do this automatically: myMidca.init()

            #print "Created MIDCA "+str(id(myMidca))+" w/ currMortarCount="+str(self.currMortarCount)

            self.myMidca = myMidca
            self.initialized = True

    def run_cycles(self, num):
        
        for cycle in range(num):
            ##print("'''''''''''''''  in run_cycles''''''''''''''''''''''''")
            self.myMidca.one_cycle(verbose = 0, pause = 0)
            #self.myMidca.display(self.myMidca.midca.world)
            ##print str(self.myMidca.midca.mem.get(self.myMidca.midca.mem.GOAL_GRAPH))
            ##print str(cycle)

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

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm

def graph():
    # get the most recent filename
    files = sorted([f for f in os.listdir(DATADIR)])
    datafile = DATADIR + files[-1]
    print("About to graph data from "+str(datafile))
    header = True
    mortar_xs = []
    cycles_ys = []
    score_zs = []
    
    with open(datafile,'r') as f:
        for line in f.readlines():
            print("line is "+str(line))
            if header: 
                header = False
            else:
                row = line.strip().split(',')
                print("row="+str(row))
                mortar_xs.append(int(row[1]))
                cycles_ys.append(int(row[3]))
                score_zs.append(int(row[4]))
        
        
        
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_trisurf(mortar_xs,cycles_ys,score_zs,cmap=cm.coolwarm)
    #ax.scatter(mortar_xs,cycles_ys,score_zs,cmap=cm.coolwarm)
    ax.legend()
    ax.set_xlabel("# Mortar")
    ax.set_ylabel("# Cycles (aka Goals)")
    ax.set_zlabel("# Score")
    plt.show()

#     fig = plt.figure()
#     ax = fig.gca(projection='3d')
#     X = mortar
#     Y = score
#     Z = cycles
#     surf = ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=cm.coolwarm,
#                        linewidth=0, antialiased=False)
#     fig.colorbar(surf, shrink=0.5, aspect=5)
# 
#     plt.show()

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == 'graph':
        # produce graph instead of running experiment
        graph()
    else:   
        MortarCogSciDemoExperiment1()

