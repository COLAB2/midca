#!/usr/bin/env python
def main():
    from midca import base
    from midca.modules import simulator, guide, evaluate, perceive, note, intend, planning, act
    from midca.metamodules import monitor, interpret, metaeval, metaintend, plan, control
    from midca.worldsim import stateread
    from midca.worldsim import pddl_num_read as pddlread
    #from midca.modules._plan.fastdown import testing_purp
    #from midca.worldsim.DIRNAME import fast_downward
    import inspect, os, pddlpy, random, fileinput, shutil,math
    from midca.worldsim import pddl,worldsim



    # Domain Specific Imports
    from midca.domains.ffdomain.minecraft import minecraft_util
    # from midca.domains.minecraft import PDDL_util

    ### this script is not working for now.



    '''
    Simulation of tower construction and arson prevention in blocksworld. Uses
    TF-trees and simulated Meta-AQUA connection to autonomously generate goals.
    '''

    thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
    GOAL_GRAPH_CMP_FUNC = minecraft_util.preferSurvive

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
    #domain and state files of correct operator
    DOMAIN_FILE = MIDCA_ROOT + "domains/ffdomain/plant_protection_domain/plant-protection-actual.pddl"
    EVENT_FILE = MIDCA_ROOT + "domains/ffdomain/plant_protection_domain/plant-protection-actual.pddl"
    STATE_FILE = MIDCA_ROOT + "domains/ffdomain/plant_protection_domain/plant_pro_actual.pddl"
    #metalevel domain and state file
    DOMAIN_FILE_M = MIDCA_ROOT + "meta_domains/plant-protection-modif.pddl"
    STATE_FILE_M = MIDCA_ROOT + "meta_domains/plant-pro_modif.pddl"



    #inputfile = open(STATE_FILE_P,'r').readlines()
    #for line in inputfile:
    #    print(line)


    '''a = list(range(1,5))
    b = 7
    print(a,b)
    ratio=0.6 #ratio of number of native plants
    nat_num = round(ratio*b)
    inv_num = b - nat_num
    nat,inv = [],[]'''
    #domain and state files of broken operator
    STATE_FILE_C = MIDCA_ROOT + "domains/ffdomain/plant_protection_domain/plant_pro_actual_2 (copy).pddl"
    DOMAIN_FILE_P = MIDCA_ROOT + "domains/ffdomain/plant_protection_domain/plant-protection-modif_nor.pddl"
    EVENT_FILE_P = MIDCA_ROOT + "domains/ffdomain/plant_protection_domain/plant-protection-modif_nor.pddl"
    STATE_FILE_P = MIDCA_ROOT + "domains/ffdomain/plant_protection_domain/plant-pro_modif.pddl"

    #opening of scenario files if you have scenarios recorded
    scenarios_file = "/home/zahid/Downloads/midca-update-to-python36/midca/examples/scenarios_60:40_100_latest_rebllion_10X10"
    f = open(scenarios_file, 'r')
    for line in f:
        line = line.replace('],[', ':')

        # a= line.split(',[')
        line = line.replace('[', '')
        line = line.replace(']', '')
        line = line.replace(' ', '')
        line = line.replace(",", '')
        a = line.split(":")
        # print(a[0])
        nat = []
        for i in range(0, int(len(a[0])), 2):
            nat.append([int(a[0][i]), int(a[0][i + 1])])
        inv = []
        for i in range(0, int(len(a[1]) - 1), 2):
            inv.append([int(a[1][i]), int(a[1][i + 1])])

        state_file_2 = shutil.copyfile(STATE_FILE,STATE_FILE_C)

        #placing the native and invasive plants in statefile by taking co-ordinates from the scenarios file
        inputfile = open(STATE_FILE_P,'r').readlines()
        write_file = open(STATE_FILE_P,'w')
        for line in inputfile:
            write_file.write(line)
            if '(and' in line:
                for i in range(len(nat)):
                    native_plant = "(native-at pos%d-%d)" %(nat[i][0],nat[i][1])
                    write_file.write(native_plant + "\n")
                for i in range(len(inv)):
                    invasive_plant = "(invasive-at pos%d-%d)" % (inv[i][0], inv[i][1])
                    write_file.write(invasive_plant + "\n")
        write_file.close()
        #writing the requested predicate to the state file inorder to get the goals from state file
        inputfile = open(state_file_2,'r').readlines()
        write_file = open(state_file_2,'w')
        for line in inputfile:
            write_file.write(line)
            if ':init' in line:
                for i in range(len(nat)):
                    native_plant = "(native-at pos%d-%d)" %(nat[i][0],nat[i][1])
                    request_1 = "(requested human1 agent1 native-at pos%d-%d )" %(nat[i][0],nat[i][1])
                    write_file.write(native_plant + "\n")
                    write_file.write(request_1 + "\n")
                for i in range(len(inv)):
                    invasive_plant = "(invasive-at pos%d-%d)" % (inv[i][0], inv[i][1])
                    request_2 = "(requested human1 agent1 not-invasive-at pos%d-%d )" % (inv[i][0], inv[i][1])
                    write_file.write(invasive_plant + "\n")
                    write_file.write(request_2 + "\n")
        write_file.close()

        #A=worldsim.Predicate('invasive-at',['?to'],[mapgrid])
        #print(A)
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
        #world1= pddlread.load_domain(DOMAIN_FILE_P, STATE_FILE_C,EVENT_FILE_P)
        #a= pddl.parseGoalDescription(world)
        #print(a)

        #print(type(world))
        #A= world.get_operators().get('move')
        #print(A)
        #print(A.postpredicates1,A.types)
        #b= [A.types[0][0]]
        #A=worldsim.Predicate('invasive-at',['?to'],b).instantiate(['?to'])
        #B= worldsim.Condition('invasive-at(?no)',b)
        #print(B)

        #print(A.name,'ji',A.objnames,'hi',A.preconditions,'hi',A.precondorder,'hi',A.prePos,'hi',A.types,'hi',A.prefunc,'hi',A.postfunc,'hi',A.prefuncnames,'hi',A.postfuncnames,'hi',A.isevent,'hi',A.results,'hi',A.resultorder,'hi',A.postPos,'hi',A.prefuncpos,'hi',A.postfunpos)


        #stateread._apply_state_pddl(world, DOMAIN_FILE, STATE_FILE)
        # creates a PhaseManager object, which wraps a MIDCA object
        myMidca = base.PhaseManager(world, display='', verbose=4, metaEnabled=True)
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
        #myMidca.append_module("Interpret", guide.MultipleGoalGen())
        myMidca.append_module("Interpret", note.GoalDiscrepancyDetector(world))
        myMidca.append_module("Eval", evaluate.SimpleEval())
        myMidca.append_module("Intend", intend.SimpleIntend())
        #myMidca.append_module("Plan", planning.MetricFFPlanner(
           # minecraft_util.ff_goals_from_midca_goals,
           #minecraft_util.ff_state_from_midca_world,
            #DOMAIN_FILE,
            #STATE_FILE))
        myMidca.append_module("Plan", planning.fastdownPlanner(
           minecraft_util.ff_goals_from_midca_goals,
            minecraft_util.ff_state_from_midca_world,
            DOMAIN_FILE_P,
            STATE_FILE_P,world))
        myMidca.append_module("Act", act.SimpleAct())
        # myMidca.insert_module('Simulate', simulator.ArrowSimulator(arrowStart=1), 1)
        # myMidca.insert_module('Simulate', simulator.AttackSimulator(skeletonStart=2), 1)
        # myMidca.insert_module('Interpret', guide.TFFire(), 2)
        #myMidca.insert_module('Interpret', guide.ReactiveSurvive(), 3)

        for phase in ["Monitor", "Interpret", "Intend", "Plan", "Control"]:
            myMidca.append_meta_phase(phase)

        myMidca.append_meta_module("Monitor", monitor.MRSimpleMonitor(nat,inv))
        myMidca.append_meta_module("Interpret", interpret.MRSimpleDetectOld())
        myMidca.append_meta_module("Interpret", interpret.MRSimpleGoalGen())
        myMidca.append_meta_module("Intend", metaintend.MRSimpleIntend())
        myMidca.append_meta_module("Plan", plan.MRfastdownPlanner(DOMAIN_FILE_M,STATE_FILE_M))
        myMidca.append_meta_module("Control", control.MRSimpleControl2())



        # Set world viewer to output text
        myMidca.set_display_function(DISPLAY_FUNC)


        # tells the PhaseManager to copy and store MIDCA states so they can be accessed later.
        myMidca.storeHistory = True
        myMidca.initGoalGraph(cmpFunc=GOAL_GRAPH_CMP_FUNC)
        myMidca.init()
        myMidca.run()

        #for i in range(30):
        #    myMidca.one_cycle_with_meta_intrlvd()
        
        # opening the results file and adding the results to it.
        results_file = "/home/zahid/Downloads/midca-update-to-python36/midca/examples/results_learned_op_60:40_10X10"
        f = open(results_file, 'a')
        f.write(str(myMidca.mem.Goal_final_achieved)+",")
        '''f.write(str(myMidca.mem.Goal_final_achieved)+",")
        f.write(str(myMidca.mem.REBEL_EVALUATION)+ ",")
            f.write(str(myMidca2.mem.get(myMidca2.mem.Goal_final_achieved_no_rebel))+ ",")
            f.write(str(myMidca3.mem.Goal_final_achieved))'''
        f.write("\n")
        f.close()
if __name__== "__main__":
  main()
'''for i in range(10):
    myMidca.one_cycle()'''

'''
The code below would print out MIDCA's goal set for the first 20 phases of the run above. Note that any memory values can be accessed in this way, assuming that the storeHistory value was set to True during the run. This code is left as an example, but commented out because it will throw an error if fewer than 20 cycles were simulated.

for i in range(20):
    print myMidca.history[i].mem.get(myMidca.midca.mem.CURRENT_GOALS)

'''
#when the state changes I need to change the domain file for the planner :O
