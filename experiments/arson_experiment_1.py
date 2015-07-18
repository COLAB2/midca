'''
Created on Jul 16, 2015

@author: dustin
'''
from MIDCA.experiment import experiment
from MIDCA import base
import datetime

class ArsonCogSciDemo():
    '''
    This is a custom experiment where MIDCA is run by varying the following:
    - probability of fire
    - using TFTrees for putoutfire goals
    - using simulated metaaqua to generate goal to put out arsonist
    '''

    def __init__(self, params):
        '''
        Setup code for experiment
        '''
        curr_datetime_str = datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d--%H-%M-%S')
        DATA_FILENAME = "ArsonCogSciDemo"+curr_datetime_str+".csv"

        # initialize the csv file here
        file = open(DATA_FILENAME, 'w')
        file.write("runID,arsonchance,usingTFTree,usingSimMA,towerscompleted,score,\n")


        def customWriteData(run_id, midca):
            '''
            This function will be called on each midca object after it has run and should get the
            necessary values (to be writtent to a data file) from the midca object
            '''
            # get the arson chance from the module
            arsonchance = -1
            for module in midca.get_modules("Simulate"):
                if module.__class__.__name__ == "ArsonSimulator":
                    arsonchance = module.getArsonChance()

            print("these are the modules")
            file.write(str(run_id)+","+"")


        ex = experiment.Experiment()


        ARSON_CHANCE_START = 0.0
        ARSON_CHANCE_END = 1.0
        ARSON_CHANCE_INCREMENT = 0.1

        NUM_CYCLES = 1000



    def createMIDCAObject(self, arsonchance, usingTFTreeFire, usingSimulatedMA):
        '''
        All the heavy duty code of creating MIDCA goes here. Use variables
        where ever you wish to parameterize.
        '''

        thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

        MIDCA_ROOT = thisDir + "/../"

        domainFile = MIDCA_ROOT + "worldsim/domains/arsonist.sim"
        stateFile = MIDCA_ROOT + "worldsim/states/defstate.sim"

        world = domainread.load_domain(domainFile)
        stateread.apply_state_file(world, stateFile)
        #creates a PhaseManager object, which wraps a MIDCA object
        myMidca = base.PhaseManager(world, display = asqiiDisplay)
        #add phases by name
        for phase in ["Simulate", "Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
            myMidca.append_phase(phase)

        #add the modules which instantiate basic blocksworld operation
        myMidca.append_module("Simulate", simulator.MidcaActionSimulator())
        myMidca.append_module("Simulate", simulator.ASCIIWorldViewer())
        myMidca.append_module("Perceive", perceive.PerfectObserver())
        myMidca.append_module("Interpret", note.ADistanceAnomalyNoter())
        myMidca.append_module("Interpret", guide.UserGoalInput())
        myMidca.append_module("Eval", evaluate.SimpleEval())
        myMidca.append_module("Intend", intend.SimpleIntend())
        myMidca.append_module("Plan", planning.PyHopPlanner(extinguish))
        myMidca.append_module("Act", act.SimpleAct())

        myMidca.insert_module('Simulate', simulator.ArsonSimulator(arsonChance, arsonStart = 10), 1)
        myMidca.insert_module('Simulate', simulator.FireReset(), 0)
        myMidca.insert_module('Interpret', guide.TFStack(), 1)

        if usingTFTreeFire:
            myMidca.insert_module('Interpret', guide.TFFire(), 2)

        if usingSimulatedMA:
            myMidca.insert_module('Interpret', guide.ReactiveApprehend(), 3)

        myMidca.insert_module('Eval', evaluate.Scorer(), 1) # this needs to be a 1 so that Scorer happens AFTER SimpleEval

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

        #tells the PhaseManager to copy and store MIDCA states so they can be accessed later.
        myMidca.storeHistory = True
        myMidca.initGoalGraph(cmpFunc = preferApprehend)
        myMidca.init()

        return myMidca
