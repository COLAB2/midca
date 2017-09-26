'''
Created on Jul 16, 2015

@author: dustin
'''
from midca.experiment.experiment import Experiment
from midca import base
from midca.worldsim import domainread, stateread, blockstate, scene
from midca.modules import simulator, perceive, note, guide, evaluate, intend, planning, act
import datetime
import os
import inspect
import time
import copy
from functools import partial

DATADIR = "experiments/ArsonCogSciDemo/data/"

def asqiiDisplay(world):
    '''
    Creates an asqii representation for blocksworld.
    '''
    blocks = blockstate.get_block_list(world)
    print str(scene.Scene(blocks))

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
        curr_datetime_str = datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d--%H-%M-%S')
        DATA_FILENAME = DATADIR + "ArsonCogSciDemo" + curr_datetime_str + ".csv"

        # initialize the csv file here
        file = open(DATA_FILENAME, 'w')
        file.write("runID,arsonchance,usingTFTree,usingSimMA,towersCompleted,score,\n")

        def customWriteData(run_id, midca):
            '''
            This function will be called on each midca object after it has run and should get the
            necessary values (to be written to a data file) from the midca object.

            Customize this function to retrieve the data you wish to collect during the experiment
            '''
            # get the arson chance from the module
            arsonchance = -1
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
            print str(midca.mem.get(evaluate.SCORE))
            file.write(str(run_id) + "," + str(arsonchance) + "," + str(usingTFTree) + "," + str(usingSimMA) + "," + str(towersCompleted) + "," + str(towersScore) + "\n")
            file.flush()

        def cleanup():
            '''
            This function is used to execute any code that needs to be run
            before closing, (i.e. closing file handles)
            '''
            file.close()

        NUM_CYCLES = 75

        ###### create code for each MIDCA run #####
        ex = Experiment(self.__class__.__name__)

        ex.addWriteDataFunc(customWriteData)
        ex.addDestructFunc(cleanup)

        ex.setNumCycles(NUM_CYCLES)
        ARSON_CHANCE_START = 0.9
        ARSON_CHANCE_END = 0.0
        ARSON_CHANCE_DECREMENT = 0.1

        # since we are varying using 3 parameters (arson chance, using tf trees, using MA)
        # we have three nested loops, creating individual midca runs for each unique paramterization

        print "Initializing each unique MIDCA run..."
        # time.sleep(0.5)
        # 1. vary by arson chance
        curr_arson_chance = ARSON_CHANCE_START
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
        print "Running each MIDCA instance..."
        # time.sleep(0.5)
        ex.run()

class MIDCAInstance():
    '''
    This class creates a specific instance of MIDCA given certain parameters.
    '''

    def __init__(self, arsonChanceArg, usingTFTreeFire, usingSimulatedMA):
        self.arsonChanceArg = arsonChanceArg
        self.usingTFTreeFire = usingTFTreeFire
        self.usingSimulatedMA = usingSimulatedMA

        self.initialized = False # to initialize, call createMIDCAObj()
        self.myMidca = None

        self.world = None

    def createMIDCAObj(self):
        # in this demo, always keep extinguish to false
            extinguish = False

            thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

            MIDCA_ROOT = thisDir + "/../"

            domainFile = MIDCA_ROOT + "worldsim/domains/arsonist.sim"
            stateFile = MIDCA_ROOT + "worldsim/states/defstate.sim"

            self.world = domainread.load_domain(domainFile)
            stateread.apply_state_file(self.world, stateFile)
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
            myMidca.append_module("Plan", planning.PyHopPlanner(extinguish))
            myMidca.append_module("Act", act.SimpleAct())

            myMidca.insert_module('Simulate', simulator.ArsonSimulator(arsonChance=self.arsonChanceArg, arsonStart=10), 1)
            myMidca.insert_module('Simulate', simulator.FireReset(), 0)
            myMidca.insert_module('Interpret', guide.TFStack(), 1)

            if self.usingTFTreeFire:
                myMidca.insert_module('Interpret', guide.TFFire(), 2)

            if self.usingSimulatedMA:
                myMidca.insert_module('Interpret', guide.ReactiveApprehend(), 3)

            myMidca.insert_module('Eval', evaluate.Scorer(), 1)  # this needs to be a 1 so that Scorer happens AFTER SimpleEval

            def preferApprehend(goal1, goal2):
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
                return 0

            # tells the PhaseManager to copy and store MIDCA states so they can be accessed later.
            myMidca.storeHistory = False
            myMidca.initGoalGraph(cmpFunc=preferApprehend)
            ## DO NOT DO THIS: experiment.py will do this automatically: myMidca.init()

            print "Created MIDCA "+str(id(myMidca))+" w/ arsonchance="+str(self.arsonChanceArg)+", usingTFTreeFire="+str(self.usingTFTreeFire)+",usingSimMA="+str(self.usingSimulatedMA)

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
        s += "\n[arsonchance]="+str(self.arsonChanceArg)
        s += "\n[usingTFTreeFire]="+str(self.usingTFTreeFire)
        s += "\n[usingSimMA]="+str(self.usingSimulatedMA)
        s += "\n[Score]="+str(self.myMidca.mem.get(evaluate.SCORE))
        return s

if __name__ == "__main__":
    ArsonCogSciDemo()

