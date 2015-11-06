'''
Created on Jul 16, 2015

@author: dustin
'''
from MIDCA.experiment.experiment import Experiment
from MIDCA import base
from MIDCA.worldsim import domainread, stateread, blockstate, scene
from MIDCA.modules import simulator, perceive, note, guide, evaluate, intend, planning, act
import datetime
import os
import inspect
import time
import copy
from functools import partial

DATADIR = "experiments/mortar-experiment-1-data/"

def asqiiDisplay(world):
    '''
    Creates an asqii representation for blocksworld.
    '''
    blocks = blockstate.get_block_list(world)
    print str(scene.Scene(blocks))

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
        curr_datetime_str = datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d--%H-%M-%S')
        DATA_FILENAME = DATADIR + "MortarCogSciDemoExperiment1" + curr_datetime_str + ".csv"

        # initialize the csv file here
        file = open(DATA_FILENAME, 'w')
        file.write("runID,numMortar,towersCompleted,score,\n")

        def customWriteData(run_id, midca):
            '''
            This function will be called on each midca object after it has run and should get the
            necessary values (to be written to a data file) from the midca object.

            Customize this function to retrieve the data you wish to collect during the experiment
            '''
            
            # get tower score
            towersCompleted = midca.mem.get(evaluate.MORTARSCORE).getTowersCompleted()
            towersScore = midca.mem.get(evaluate.MORTARSCORE).getTowersScore()
            print str(midca.mem.get(evaluate.MORTARSCORE))
            numMortars = midca.mem.get(evaluate.MORTARSCORE).getMortarBlocks()
            file.write(str(run_id) + "," + str(numMortars) + "," + str(towersCompleted) + "," + str(towersScore) + "\n")
            file.flush()

        def cleanup():
            '''
            This function is used to execute any code that needs to be run
            before closing, (i.e. closing file handles)
            '''
            file.close()
        
        NUM_CYCLES = 200

        ###### create code for each MIDCA run #####
        ex = Experiment(self.__class__.__name__)

        ex.addWriteDataFunc(customWriteData)
        ex.addDestructFunc(cleanup)

        ex.setNumCycles(NUM_CYCLES)
        MORTAR_QUANTITY_START = 1
        MORTAR_QUANTITY_END = 100
        MORTAR_QUANTITY_INCREMENT = 1 # this should ideally be a function
        
        # since we are varying using 3 parameters (arson chance, using tf trees, using MA)
        # we have three nested loops, creating individual midca runs for each unique paramterization

        print "Initializing each unique MIDCA run..."
        # time.sleep(0.5)
        # 1. vary by arson chance
        curr_mortar_count = MORTAR_QUANTITY_START
        while curr_mortar_count > MORTAR_QUANTITY_END:
            # create MIDCA instance
            midcaInst = MIDCAInstance(curr_mortar_count)
            midcaInst.createMIDCAObj()
            ex.appendRun(midcaInst)

            curr_mortar_count += MORTAR_QUANTITY_INCREMENT
        print "Running each MIDCA instance..."
        # time.sleep(0.5)


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
            state_lines = open(stateFile).readlines() # first read file
            # now add new mortar blocks
            for i in range(self.currMortarCount):
                state_lines.append("MORTARBLOCK(M"+str(i)+")\n")
                state_lines.append("available(M"+str(i)+")\n")
            # now load the state    
            stateread.apply_state_str(self.world, state_lines)
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

            print "Created MIDCA "+str(id(myMidca))+" w/ currMortarCount="+str(self.currMortarCount)

            self.myMidca = myMidca
            self.initialized = True

    def run_cycles(self, num):
        for cycle in range(num):
                self.myMidca.one_cycle(verbose = 0, pause = 0)
                #self.myMidca.display(self.myMidca.midca.world)
                #print str(self.myMidca.midca.mem.get(self.myMidca.midca.mem.GOAL_GRAPH))
                #print str(cycle)

    def getMIDCAObj(self):
        return self.myMidca

    def __str__(self):
        s = "MIDCAInstance [id]="+str(id(self.myMidca))
        s += "\n[currMortarCount]="+str(self.currMortarCount)
        s += "\n[Score]="+str(self.myMidca.mem.get(evaluate.MORTARSCORE))
        return s

if __name__ == "__main__":
    MortarCogSciDemoExperiment1()

