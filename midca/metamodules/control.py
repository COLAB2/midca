

import os, copy,re
import importlib
from midca import base, goals, modules
from midca.modules.gens import goaltransform
from midca.metamodules.foil_python_master.src.trimlogic.test import FamilyTreeTestCase_1

class MRSimpleControl(base.BaseModule):
    midca = None
    prev_init_args = []  # TODO: fix, right now used only for planning and saving args

    # from a remove module action and then using the args to
    # insert a new module action

    def run(self, cycle, verbose=2):
        self.verbose = verbose
        plan = self.mem.get(self.mem.META_PLAN)
        self.prev_init_args = []
        failed = False
        if plan:
            for action in plan:
                if not self.act(action):
                    failed = True
                    if self.verbose >= 1: print("    MRSimpleControl failed to complete an action")
                    self.cleanup_and_reset()
            if not failed:
                self.cleanup_and_reset()
        else:
            if self.verbose >= 1: print("    No actions taken")

    def cleanup_and_reset(self):
        """
        Temporary function to remove goal and plan,
        should eventually be replaced by evaluate
         """
        self.mem.set(self.mem.META_PLAN, None)
        self.mem.set(self.mem.META_GOALS, None)
        self.mem.set(self.mem.META_CURR_GOAL, None)
        self.mem.set(self.mem.PLAN, [])

    def act(self, action, verbose=2):
        # TODO: figure out a way to make the init_args more general (so actions can be kept separate)
        if action[0] == "REMOVE-MODULE":
            # find the component
            module_index = -1
            phase = None
            mod_str = ""

            class Found(Exception):
                pass  # is this bad python? should this go at top of my file?

            try:
                for phasei in self.mem.myMidca.get_phases():
                    i = 0
                    for mod in self.mem.myMidca.get_modules(phasei):
                        mod_str = str(mod.__class__.__name__)
                        print(("-*-*- act():  mod = " + mod_str + ", action[1] = " + str(action[1])))
                        if mod_str == action[1]:
                            print("-*-*- act(): we got a match!")
                            module_index = i
                            phase = phasei
                            raise Found
                        i += 1
            except Found:

                # remove the component
                # print("-*-*- act():  phase = "+str(phase)+", module_index = "+str(module_index))
                if phase and module_index > -1:
                    mod = self.mem.myMidca.remove_module(phase, module_index)
                    print("mod is " + str(mod))
                    self.prev_init_args = mod.get_init_args()  # TODO: only works for modules implementing get_init_args
                    print("got init args: " + str(self.prev_init_args))
                    is_success = mod_str not in [x.__class__.__name__ for x in self.mem.myMidca.get_modules(phase)]
                    if is_success: print(
                        ("    Metareasoner removed " + mod_str))  # report any actions metareasoner carried out
                    return is_success
        elif action[0] == "ADD-MODULE":
            if action[2] == "PyHopPlanner":
                # print("current directory: "+str(os.getcwd()))
                # print("str(dir(modules)) = "+str(dir(modules)))
                planningModuleInstance = importlib.import_module("midca.modules.planning")
                print(("loaded planning module, it has following attributes: " + str(dir(planningModuleInstance))))
                # get the args used to init the old module and use them to init this one
                print("init args is " + str(self.prev_init_args))

                # **** BEGIN: MAGIC! Transform the args *****
                # This is where the real magic happens
                # Hardcoded for now
                # Very Important TODO but likely requires serious research effort
                from midca.domains.blocksworld.plan import methods
                working_methods = methods.declare_methods
                corrected_args = self.prev_init_args
                hardcoded_index_of_methods_arg = 2  # TODO: I know, so hacky, ahhh magic numbers
                corrected_args[2] = working_methods
                # **** END: MAGIC! Transform the args *****

                pyHopPlannerInstance = planningModuleInstance.PyHopPlanner(*corrected_args)
                self.mem.myMidca.runtime_append_module("Plan",
                                                       pyHopPlannerInstance)  # TODO: hardcoded knowledge of Plan phase
                is_success = "PyHopPlanner" in [x.__class__.__name__ for x in self.mem.myMidca.get_modules("Plan")]
                if is_success: print(
                    "    Metareasoner added PyHopPlanner")  # report any actions metareasoner carried out
                return is_success
            elif action[2] == "AsynchPyhopPlanner":
                # print("current directory: "+str(os.getcwd()))
                # print("str(dir(modules)) = "+str(dir(modules)))
                planningModuleInstance = importlib.import_module("midca.modules.planning")
                print(("loaded asynchronous planning module, it has following attributes: " + str(
                    dir(planningModuleInstance))))
                # get the args used to init the old module and use them to init this one
                # print "init args is "+str(self.prev_init_args)
                ###### HACK: hardcoded for now as a demo, this is because the initial broken
                # planner had different parameters for instantiating it, so we cant use the\
                # same prev_init_args, and thus changing them here
                from midca.modules._plan.asynch import operators_sr, methods_sr
                self.prev_init_args = [methods_sr.declare_methods, operators_sr.declare_ops]

                # **** BEGIN: MAGIC! Transform the args *****
                # This is where the real magic happens
                # Hardcoded for now
                # Very Important TODO but likely requires serious research effort
                # from midca.domains.blocksworld.plan import methods
                # working_methods = methods.declare_methods
                corrected_args = self.prev_init_args
                # hardcoded_index_of_methods_arg = 2 # TODO: I know, so hacky, ahhh magic numbers
                # corrected_args[2] = working_methods
                # **** END: MAGIC! Transform the args *****

                pyHopPlannerInstance = planningModuleInstance.AsynchPyhopPlanner(*corrected_args)
                self.mem.myMidca.runtime_append_module("Plan",
                                                       pyHopPlannerInstance)  # TODO: hardcoded knowledge of Plan phase
                is_success = "AsynchPyhopPlanner" in [x.__class__.__name__ for x in
                                                      self.mem.myMidca.get_modules("Plan")]
                if is_success: print(
                    "    Metareasoner added AsynchPyhopPlanner")  # report any actions metareasoner carried out
                return is_success
        elif action[0] == "TRANSFORM-GOAL":
            # really: its going to have this meta plan by changing the things in orange - 
            # you don't monitor and look at the trace to see if the goal transformation achieved something
            # at the ground level, so then meta-evaluate can know if the metagoal succeeded. What if I made the
            # wrong transformation? 

            # specific to mortar and blocks world, not general
            # get the highest blocks
            goal_atoms = action[1]
            goal_args = [x.get_args() for x in goal_atoms]

            # figure out how much mortar we have using the world state
            num_available_mortar = 0
            for atom in map(str, self.mem.get("__world states")[-1].get_atoms()):
                if 'available(' in atom:
                    num_available_mortar += 1
                    if verbose >= 3: print(("found available mortar: " + str(atom)))

            # transform 'stable-on' to 'on' predicates ensuring there is enough mortar for each
            bottom_blks = []
            top_blks = []
            for a_tpl in goal_args:
                bottom_blks.append(a_tpl[1])
                top_blks.append(a_tpl[0])

            bottommost_blk = [b for b in bottom_blks if b not in top_blks][0]

            transformed_goal_atoms = []

            # transform goal atoms
            curr_bottom_blk = bottommost_blk
            while curr_bottom_blk in bottom_blks:
                i = bottom_blks.index(curr_bottom_blk)
                top_blks[i]
                if num_available_mortar > 0:
                    transformed_goal_atoms.append("stable-on," + top_blks[i] + "," + curr_bottom_blk)
                    num_available_mortar -= 1
                else:
                    transformed_goal_atoms.append("on," + top_blks[i] + "," + curr_bottom_blk)
                curr_bottom_blk = top_blks[i]

            # now make actual MIDCA goal objects
            transformed_goals = []
            for atom_str in transformed_goal_atoms:
                vals = atom_str.split(",")
                transformed_goals.append(goals.Goal(*[vals[1], vals[2]], predicate=vals[0]))

            goalGraph = self.mem.get(self.mem.GOAL_GRAPH)

            # do removal first
            for g in goal_atoms:
                goalGraph.remove(g)
            if verbose >= 2:
                # now display success statement
                print("Removed from the goal graph the previous goal:")
                for g in goal_atoms:
                    print(("    " + str(g)))

            # now display transformed goal
            if verbose >= 2:
                print("Goal has been transformed to:")
                for g in transformed_goals:
                    print(("    " + str(g)))

            # now insert the new goal
            for g in transformed_goals:
                goalGraph.insert(g)
            if verbose >= 2:
                print("Transformed goal has been inserted into goal graph.")

            return True
            # goals.Goal(*['A_','B_'], predicate = 'stable-on'),
            #       goals.Goal(*['C_','A_'], predicate = 'stable-on'),
            #       goals.Goal(*['D_','C_'], predicate = 'stable-on')

            # and go top down
            # first, get the heights


class MRSimpleControl1(base.BaseModule):
    midca = None
    '''
    def init(self, world):
        if not world:
            raise ValueError("world is None!")
        self.world = world
    '''

    def run(self, cycle, verbose=2):
        plan = self.mem.get(self.mem.META_PLAN)
        failed = False
        if plan:
            for action in plan:
                if not self.act(action):
                    failed = True
                    if verbose >= 2: print("    MRSimpleControl failed to complete an action")
                    self.cleanup_and_reset()
            if not failed:
                self.cleanup_and_reset()
        else:
            if verbose >= 2: print("    No actions taken")

    def cleanup_and_reset(self):
        """
        Temporary function to remove goal and plan,
        should eventually be replaced by evaluate
         """
        self.mem.set(self.mem.META_PLAN, None)
        self.mem.set(self.mem.META_GOALS, None)
        self.mem.set(self.mem.META_CURR_GOAL, None)

    def act(self, action, verbose=2):
        if action[0] == "REMOVE-MODULE":
            # find the component
            module_index = -1
            phase = None
            mod_str = ""

            class Found(Exception):
                pass  # is this bad python? should this go at top of my file?

            try:
                for phasei in self.mem.myMidca.get_phases():
                    i = 0
                    for mod in self.mem.myMidca.get_modules(phasei):
                        mod_str = str(mod.__class__.__name__)
                        # print("-*-*- act():  mod = "+mod_str+", action[1] = "+str(action[1]))
                        if mod_str == action[1]:
                            # print("-*-*- act(): we got a match!")
                            module_index = i
                            phase = phasei
                            raise Found
                        i += 1
            except Found:

                # remove the component
                # print("-*-*- act():  phase = "+str(phase)+", module_index = "+str(module_index))
                if phase and module_index > -1:
                    self.mem.myMidca.remove_module(phase, module_index)
                    is_success = mod_str not in [x.__class__.__name__ for x in self.mem.myMidca.get_modules(phase)]
                    if is_success: print(
                        ("    Metareasoner removed " + mod_str))  # report any actions metareasoner carried out
                    return is_success
        elif action[0] == "ADD-MODULE":
            if action[2] == "PyHopPlanner":
                # print("current directory: "+str(os.getcwd()))
                planningModuleInstance = __import__("modules.planning")
                # print("loaded planning module, it has following attributes: "+str(dir(planningModuleInstance)))
                pyHopPlannerInstance = planningModuleInstance.planning.PyHopPlanner(True)
                self.mem.myMidca.runtime_append_module("Plan",
                                                       pyHopPlannerInstance)  # TODO: hardcoded knowledge of Plan phase
                is_success = "PyHopPlanner" in [x.__class__.__name__ for x in self.mem.myMidca.get_modules("Plan")]
                if is_success: print(
                    "    Metareasoner added PyHopPlanner")  # report any actions metareasoner carried out
                return is_success
        elif action[0] == "TRANSFORM-GOAL":
            # really: its going to have this meta plan by changing the things in orange - 
            # you don't monitor and look at the trace to see if the goal transformation achieved something
            # at the ground level, so then meta-evaluate can know if the metagoal succeeded. What if I made the
            # wrong transformation? 
            # print(self.world.cltree.rootnode.predicate)
            # specific to mortar and blocks world, not general
            # get the highest blocks
            # goal_check is the copy of goal graph intended to check the goalgraph goals and delete the goals in the goal graph
            goal_check = copy.deepcopy(action[1])
            goal_atoms = action[1]
            goal_args = [x.get_args() for x in goal_atoms]
            mem = self.mem
            # get the world
            world = self.mem.myMidca.midca.world
            # initialize the resources for goal transform
            goaltransform.resources(mem)
            transformed_goals = []
            goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
            for i in range(0, len(goal_atoms)):
                trans_goal = goaltransform.choose(mem, goal_atoms[i])
                transformed_goals.append(trans_goal)
                world.cltree['checked'] = []

            # do removal first
            for g in goal_check:
                goalGraph.remove(g)

            if verbose >= 2:
                # now display success statement
                print("Removed from the goal graph the previous goal:")
                for g in goal_check:
                    print(("    " + str(g)))

            # now display transformed goal
            if verbose >= 2:
                print("Goal has been transformed to:")
                for g in transformed_goals:
                    print(("    " + str(g)))

            # now insert the new goal
            for g in transformed_goals:
                goalGraph.insert(g)

            self.mem.set(self.mem.CURRENT_GOALS, transformed_goals)
            if verbose >= 2:
                print("Transformed goal has been inserted into goal graph.")

            return True
            # goals.Goal(*['A_','B_'], predicate = 'stable-on'),
            #       goals.Goal(*['C_','A_'], predicate = 'stable-on'),
            #       goals.Goal(*['D_','C_'], predicate = 'stable-on')

            # and go top down
            # first, get the heights

from midca.worldsim.pddl import Predicate
import inspect, os, random, fileinput, shutil,math
from midca.worldsim import pddl_num_read as pddlread
from midca.worldsim import worldsim
class MRSimpleControl2(base.BaseModule):
    midca = None
    prev_init_args = []  # TODO: fix, right now used only for planning and saving args

    # from a remove module action and then using the args to
    # insert a new module action
    '''def __init__(self,b,world1):
        self.types = b
        self.world = world1'''

    def run(self, cycle, verbose=2):
        thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
        MIDCA_ROOT = thisDir + "/../"
        DOMAIN_FILE_P = MIDCA_ROOT + "domains/ffdomain/plant_protection_domain/plant-protection-modif_nor.pddl"
        #DOMAIN_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft1/plant-protection-modif_nor_2.pddl"
        rule_file = "/home/zahid/Downloads//foil-python-master/src/trimlogic/test/rule.txt"
        print(self.mem.get(self.mem.META_PLAN))
        if (self.mem.get(self.mem.META_PLAN)):
            #print(self.mem.META_PLAN)
            #A = self.world.get_operators().get('spray').results
            #inputfile = open(DOMAIN_FILE, 'w+').readlines()
            #write_file = open(DOMAIN_FILE_P, 'w')
            dicto = {'south(':'?so','north(':'?no','east(':'?ea','west(':'?we','noes(':'?noes','nows(':'?nows','soes(':'?soes','sows(':'?sows'}
            dicto1 = {'south':'?so','north':'?no','east':'?ea','west':'?we','noes':'?noes','nows':'?nows','soes':'?soes','sows':'?sows'}
            reverse = {'east': 'west', 'west': 'east', 'north': 'south', 'south': 'north',
                       'noes': 'sows', 'sows': 'noes', 'nows': 'soes', 'soes': 'nows'}
            #os.chdir('/home/zahid/Downloads//foil-python-master/src/trimlogic/test')
            #import FamilyTreeTestCase_1 as foil
            #foil
            #os.system('python -m unittest trimlogic.test.FamilyTreeTestCase_1')
            FamilyTreeTestCase_1
            #f = open(rule_file, "r")
            f = open(rule_file, 'r').readlines()
            for line in f:
                #matches = re.findall("[a-zA-Z_]+\(", line)
                line= line.replace('(',',')
                line= line.replace(')','')
                #print(line)
                line = line.replace(', ', ',')
                line = line.replace('ISADJ_', '')
                line=line.replace('.\n','')
                line = line.split(',')
                #print(line)
                try:
                    r = line.index('spray')
                    b = []

                    if len(line) == 9:

                        if line[r + 1] in line[r + 3:]:
                            if line[r + 3:].index(line[r + 1]) % 3 == 1:
                                b.append(line[r + 2 + (line[r + 3:].index(line[r + 1]))])
                            elif line[r + 3:].index(line[r + 1]) % 3 == 2:

                                b.append(reverse[line[r + 1 + (line[r + 3:].index(line[r + 1]))]])

                        input_file = open(DOMAIN_FILE_P, 'r').readlines()
                        write_file = open(DOMAIN_FILE_P, 'w')
                        flag = 0
                        for line in input_file:
                            if '(:action spray' in line:
                                flag = 1
                                write_file.write('(:action spray\n')
                            elif '    :effect (and\n' in line and flag == 1:
                                write_file.write('    :effect (and\n')
                                write_file.write(
                                    '(when (ISADJ ' + b[0] + ' ?to ' + dicto1[b[0]] + ') (and (not (native-at ' + dicto1[
                                        b[0]] + '))(not (invasive-at ' + dicto1[b[0]] + ')) ))\n')
                                flag = 0
                            else:
                                write_file.write(line)
                        write_file.close()
                    elif len(line) == 12:
                        c=[]
                        d=0
                        if line[r + 1] in line[r + 3:]:
                            if line[r + 3:].index(line[r + 1]) % 3 == 1:
                                # print(line[r + 3:].index(line[r + 1]))
                                b.append(line[r + 2 + (line[r + 3:].index(line[r + 1]))])
                                d= line[r + 4 + (line[r + 3:].index(line[r + 1]))]
                                c.append(line[(line.index(d))-1])
                                #print(b,c)


                            elif line[r + 3:].index(line[r + 1]) % 3 == 2:

                                b.append(reverse[line[r + 1 + (line[r + 3:].index(line[r + 1]))]])
                                d= line[r + 2 + (line[r + 3:].index(line[r + 1]))]
                                c.append(line[(line.index(d))-1])

                                #print(b,c)
                            input_file = open(DOMAIN_FILE_P, 'r').readlines()
                            write_file = open(DOMAIN_FILE_P, 'w')
                            flag = 0
                            for line in input_file:
                                if '(:action spray' in line:
                                    flag = 1
                                    write_file.write('(:action spray\n')
                                elif '    :effect (and\n' in line and flag == 1:
                                    write_file.write('    :effect (and\n')
                                    write_file.write(
                                        '(forall (?pos - mapgrid)'+'(when (and (ISADJ ' + b[0] + ' ?to ' + dicto1[b[0]] + ')' +'(ISADJ '+ c[0]+' '+ dicto1[b[0]]+' ?pos))'+ '(and (not (native-at ?pos))(not (invasive-at ?pos)) )))\n')
                                    flag = 0
                                else:
                                    write_file.write(line)
                            write_file.close()
                except:
                    pass
                #print('jijijiji',b)
                #l = line.index(c)
                #if l % 3 == 1:
                #    b.append(line[l - 1])
                #elif l % 3 == 2:
                #    b.append(reverse[line[l - 2]])
                #print(line,b)




                #print(matches)
                '''for x in matches:
                    a=x.split('_')
                    try:
                        if a[1] in ['south(','north(','west(','east(','noes(','nows(','soes(','sows(']:
                            input_file = open(DOMAIN_FILE_P, 'r').readlines()
                            write_file = open(DOMAIN_FILE_P, 'w')
                            flag=0
                            for line in input_file:
                                if '(:action spray' in line:
                                    flag = 1
                                    write_file.write('(:action spray\n')
                                elif '    :effect (and\n' in line and flag == 1:
                                    write_file.write('    :effect (and\n')
                                    write_file.write('(when (ISADJ '+a[1][:-1]+ ' ?to '+ dicto[a[1]]+') (and (not (native-at '+dicto[a[1]]+'))(not (invasive-at '+dicto[a[1]]+')) ))\n')
                                    flag=0
                                else:
                                    write_file.write(line)
                            write_file.close()
                        #shutil.copyfile(write_file, inputfile)
                    except:
                        pass'''



            '''cond = worldsim.Condition('invasive-at(?no)',self.types)
            cond1 = worldsim.Condition('invasive-at(?so)',self.types)
            cond2 = worldsim.Condition('invasive-at(?ea)', self.types)
            cond3 = worldsim.Condition('invasive-at(?we)', self.types)
            A[cond] = ['?no']
            A[cond1] = ['?so']
            A[cond2] = ['?ea']
            A[cond3] = ['?we']'''
            '''inputfile = open(DOMAIN_FILE_P, 'r').readlines()
            writefile = open(DOMAIN_FILE_P, 'w')
            flag_1=0
            flag_2=0
            for line in inputfile:
                writefile.write(line)
                if '(:action spray' in line:
                    flag_1 =1
                if flag_1 == 1 and '    :effect' in line:
                    flag_2 = 1
                if flag_2 == 1 and '    (and' in line:
                    #writefile.write("        dvsdv)"+ "\n")
                    writefile.write("        (not (invasive-at ?no))"+ "\n")
                    writefile.write("        (not (invasive-at ?so))"+ "\n")
                    writefile.write("        (not (invasive-at ?ea))" + "\n")
                    writefile.write("        (not (invasive-at ?we))" + "\n")
                    writefile.write("        (not (invasive-at ?noes))" + "\n")
                    writefile.write("        (not (invasive-at ?sows))" + "\n")
                    writefile.write("        (not (invasive-at ?nows))" + "\n")
                    writefile.write("        (not (invasive-at ?soes))" + "\n")
                    writefile.write("        (not (native-at ?no))" + "\n")
                    writefile.write("        (not (native-at ?so))" + "\n")
                    writefile.write("        (not (native-at ?ea))" + "\n")
                    writefile.write("        (not (native-at ?we))" + "\n")
                    writefile.write("        (not (native-at ?noes))" + "\n")
                    writefile.write("        (not (native-at ?sows))" + "\n")
                    writefile.write("        (not (native-at ?nows))" + "\n")
                    writefile.write("        (not (native-at ?soes))" + "\n")'''
            plan=[]
            self.mem.set(self.mem.META_PLAN, plan)
            self.mem.DISCREPANCY = False
            self.mem.learning_evaluation = True
            #print("cdacac",self.world.get_operators().get('spray').results)


        '''for line in inputfile:
            writefile.write(line)
            if '(and' in line:
                for i in range(nat_num):
                    native_plant = "(native-at pos%d-%d)" % (nat[i][0], nat[i][1])
                    writefile.write(native_plant + "\n")'''

        '''for i  in range(len(inputfile)):
            writefile.write(inputfile[i])
            if '(:action spray1' in inputfile[i]:
                for j in range(len(inputfile[i:])):
                    if '    :effect' in inputfile[i+j]:
                        writefile.write("hei")
                        break'''
        '''writefile.close()'''


        self.verbose = verbose
        #plan = self.mem.get(self.mem.META_PLAN)

        self.prev_init_args = []
        failed = False
        '''print(world1.get_operators().get('spray').objnames)'''
        #for a in dom.actions:



        '''if plan:
            for action in plan:
                if not self.act(action):
                    failed = True
                    if self.verbose >= 1: print("    MRSimpleControl failed to complete an action")
                    self.cleanup_and_reset()
            if not failed:
                self.cleanup_and_reset()
        else:
            if self.verbose >= 1: print("    No actions taken")'''

    def cleanup_and_reset(self):
        """
        Temporary function to remove goal and plan,
        should eventually be replaced by evaluate
         """
        self.mem.set(self.mem.META_PLAN, None)
        self.mem.set(self.mem.META_GOALS, None)
        self.mem.set(self.mem.META_CURR_GOAL, None)
        self.mem.set(self.mem.PLAN, [])

    def act(self, action, verbose=2):
        # TODO: figure out a way to make the init_args more general (so actions can be kept separate)
        if action[0] == "REMOVE-MODULE":
            # find the component
            module_index = -1
            phase = None
            mod_str = ""

            class Found(Exception):
                pass  # is this bad python? should this go at top of my file?

            try:
                for phasei in self.mem.myMidca.get_phases():
                    i = 0
                    for mod in self.mem.myMidca.get_modules(phasei):
                        mod_str = str(mod.__class__.__name__)
                        print(("-*-*- act():  mod = " + mod_str + ", action[1] = " + str(action[1])))
                        if mod_str == action[1]:
                            print("-*-*- act(): we got a match!")
                            module_index = i
                            phase = phasei
                            raise Found
                        i += 1
            except Found:

                # remove the component
                # print("-*-*- act():  phase = "+str(phase)+", module_index = "+str(module_index))
                if phase and module_index > -1:
                    mod = self.mem.myMidca.remove_module(phase, module_index)
                    print()
                    "mod is " + str(mod)
                    self.prev_init_args = mod.get_init_args()  # TODO: only works for modules implementing get_init_args
                    print()
                    "got init args: " + str(self.prev_init_args)
                    is_success = mod_str not in [x.__class__.__name__ for x in self.mem.myMidca.get_modules(phase)]
                    if is_success: print((
                        "    Metareasoner removed " + mod_str))  # report any actions metareasoner carried out
                    return is_success
        elif action[0] == "ADD-MODULE":
            if action[2] == "PyHopPlanner":
                # print("current directory: "+str(os.getcwd()))
                # print("str(dir(modules)) = "+str(dir(modules)))
                planningModuleInstance = importlib.import_module("midca.modules.planning")
                print(("loaded planning module, it has following attributes: " + str(dir(planningModuleInstance))))
                # get the args used to init the old module and use them to init this one
                print()
                "init args is " + str(self.prev_init_args)

                # **** BEGIN: MAGIC! Transform the args *****
                # This is where the real magic happens
                # Hardcoded for now
                # Very Important TODO but likely requires serious research effort
                from midca.domains.blocksworld.plan import methods
                working_methods = methods.declare_methods
                corrected_args = self.prev_init_args
                hardcoded_index_of_methods_arg = 2  # TODO: I know, so hacky, ahhh magic numbers
                corrected_args[2] = working_methods
                # **** END: MAGIC! Transform the args *****

                pyHopPlannerInstance = planningModuleInstance.PyHopPlanner(*corrected_args)
                self.mem.myMidca.runtime_append_module("Plan",
                                                       pyHopPlannerInstance)  # TODO: hardcoded knowledge of Plan phase
                is_success = "PyHopPlanner" in [x.__class__.__name__ for x in self.mem.myMidca.get_modules("Plan")]
                if is_success: print(
                    "    Metareasoner added PyHopPlanner")  # report any actions metareasoner carried out
                return is_success
            elif action[2] == "AsynchPyhopPlanner":
                # print("current directory: "+str(os.getcwd()))
                # print("str(dir(modules)) = "+str(dir(modules)))
                planningModuleInstance = importlib.import_module("midca.modules.planning")
                print(("loaded asynchronous planning module, it has following attributes: " + str(
                    dir(planningModuleInstance))))
                # get the args used to init the old module and use them to init this one
                # print "init args is "+str(self.prev_init_args)
                ###### HACK: hardcoded for now as a demo, this is because the initial broken
                # planner had different parameters for instantiating it, so we cant use the\
                # same prev_init_args, and thus changing them here
                from midca.modules._plan.asynch import operators_sr, methods_sr
                self.prev_init_args = [methods_sr.declare_methods, operators_sr.declare_ops]

                # **** BEGIN: MAGIC! Transform the args *****
                # This is where the real magic happens
                # Hardcoded for now
                # Very Important TODO but likely requires serious research effort
                # from midca.domains.blocksworld.plan import methods
                # working_methods = methods.declare_methods
                corrected_args = self.prev_init_args
                # hardcoded_index_of_methods_arg = 2 # TODO: I know, so hacky, ahhh magic numbers
                # corrected_args[2] = working_methods
                # **** END: MAGIC! Transform the args *****

                pyHopPlannerInstance = planningModuleInstance.AsynchPyhopPlanner(*corrected_args)
                self.mem.myMidca.runtime_append_module("Plan",
                                                       pyHopPlannerInstance)  # TODO: hardcoded knowledge of Plan phase
                is_success = "AsynchPyhopPlanner" in [x.__class__.__name__ for x in self.mem.myMidca.get_modules("Plan")]
                if is_success: print(
                    "    Metareasoner added AsynchPyhopPlanner")  # report any actions metareasoner carried out
                return is_success
        elif action[0] == "TRANSFORM-GOAL":
            # really: its going to have this meta plan by changing the things in orange -
            # you don't monitor and look at the trace to see if the goal transformation achieved something
            # at the ground level, so then meta-evaluate can know if the metagoal succeeded. What if I made the
            # wrong transformation?

            # specific to mortar and blocks world, not general
            # get the highest blocks
            goal_atoms = action[1]
            goal_args = [x.get_args() for x in goal_atoms]

            # figure out how much mortar we have using the world state
            num_available_mortar = 0
            for atom in map(str, self.mem.get("__world states")[-1].get_atoms()):
                if 'available(' in atom:
                    num_available_mortar += 1
                    if verbose >= 3: print(("found available mortar: " + str(atom)))

            # transform 'stable-on' to 'on' predicates ensuring there is enough mortar for each
            bottom_blks = []
            top_blks = []
            for a_tpl in goal_args:
                bottom_blks.append(a_tpl[1])
                top_blks.append(a_tpl[0])

            bottommost_blk = [b for b in bottom_blks if b not in top_blks][0]

            transformed_goal_atoms = []

            # transform goal atoms
            curr_bottom_blk = bottommost_blk
            while curr_bottom_blk in bottom_blks:
                i = bottom_blks.index(curr_bottom_blk)
                top_blks[i]
                if num_available_mortar > 0:
                    transformed_goal_atoms.append("stable-on," + top_blks[i] + "," + curr_bottom_blk)
                    num_available_mortar -= 1
                else:
                    transformed_goal_atoms.append("on," + top_blks[i] + "," + curr_bottom_blk)
                curr_bottom_blk = top_blks[i]

            # now make actual MIDCA goal objects
            transformed_goals = []
            for atom_str in transformed_goal_atoms:
                vals = atom_str.split(",")
                transformed_goals.append(goals.Goal(*[vals[1], vals[2]], predicate=vals[0]))

            goalGraph = self.mem.get(self.mem.GOAL_GRAPH)

            # do removal first
            for g in goal_atoms:
                goalGraph.remove(g)
            if verbose >= 2:
                # now display success statement
                print("Removed from the goal graph the previous goal:")
                for g in goal_atoms:
                    print(("    " + str(g)))

            # now display transformed goal
            if verbose >= 2:
                print("Goal has been transformed to:")
                for g in transformed_goals:
                    print(("    " + str(g)))

            # now insert the new goal
            for g in transformed_goals:
                goalGraph.insert(g)
            if verbose >= 2:
                print("Transformed goal has been inserted into goal graph.")

            return True
            # goals.Goal(*['A_','B_'], predicate = 'stable-on'),
            #       goals.Goal(*['C_','A_'], predicate = 'stable-on'),
            #       goals.Goal(*['D_','C_'], predicate = 'stable-on')

            # and go top down
            # first, get the heights


class MRSimpleControl1(base.BaseModule):
    midca = None
    '''
    def init(self, world):
        if not world:
            raise ValueError("world is None!")
        self.world = world
    '''

    def run(self, cycle, verbose=2):
        plan = self.mem.get(self.mem.META_PLAN)
        failed = False
        if plan:
            for action in plan:
                if not self.act(action):
                    failed = True
                    if verbose >= 2: print("    MRSimpleControl failed to complete an action")
                    self.cleanup_and_reset()
            if not failed:
                self.cleanup_and_reset()
        else:
            if verbose >= 2: print("    No actions taken")

    def cleanup_and_reset(self):
        """
        Temporary function to remove goal and plan,
        should eventually be replaced by evaluate
         """
        self.mem.set(self.mem.META_PLAN, None)
        self.mem.set(self.mem.META_GOALS, None)
        self.mem.set(self.mem.META_CURR_GOAL, None)

    def act(self, action, verbose=2):
        if action[0] == "REMOVE-MODULE":
            # find the component
            module_index = -1
            phase = None
            mod_str = ""

            class Found(Exception):
                pass  # is this bad python? should this go at top of my file?

            try:
                for phasei in self.mem.myMidca.get_phases():
                    i = 0
                    for mod in self.mem.myMidca.get_modules(phasei):
                        mod_str = str(mod.__class__.__name__)
                        # print("-*-*- act():  mod = "+mod_str+", action[1] = "+str(action[1]))
                        if mod_str == action[1]:
                            # print("-*-*- act(): we got a match!")
                            module_index = i
                            phase = phasei
                            raise Found
                        i += 1
            except Found:

                # remove the component
                # print("-*-*- act():  phase = "+str(phase)+", module_index = "+str(module_index))
                if phase and module_index > -1:
                    self.mem.myMidca.remove_module(phase, module_index)
                    is_success = mod_str not in [x.__class__.__name__ for x in self.mem.myMidca.get_modules(phase)]
                    if is_success: print((
                        "    Metareasoner removed " + mod_str))  # report any actions metareasoner carried out
                    return is_success
        elif action[0] == "ADD-MODULE":
            if action[2] == "PyHopPlanner":
                # print("current directory: "+str(os.getcwd()))
                planningModuleInstance = __import__("modules.planning")
                # print("loaded planning module, it has following attributes: "+str(dir(planningModuleInstance)))
                pyHopPlannerInstance = planningModuleInstance.planning.PyHopPlanner(True)
                self.mem.myMidca.runtime_append_module("Plan",
                                                       pyHopPlannerInstance)  # TODO: hardcoded knowledge of Plan phase
                is_success = "PyHopPlanner" in [x.__class__.__name__ for x in self.mem.myMidca.get_modules("Plan")]
                if is_success: print(
                    "    Metareasoner added PyHopPlanner")  # report any actions metareasoner carried out
                return is_success
        elif action[0] == "TRANSFORM-GOAL":
            # really: its going to have this meta plan by changing the things in orange -
            # you don't monitor and look at the trace to see if the goal transformation achieved something
            # at the ground level, so then meta-evaluate can know if the metagoal succeeded. What if I made the
            # wrong transformation?
            # print(self.world.cltree.rootnode.predicate)
            # specific to mortar and blocks world, not general
            # get the highest blocks
            # goal_check is the copy of goal graph intended to check the goalgraph goals and delete the goals in the goal graph
            goal_check = copy.deepcopy(action[1])
            goal_atoms = action[1]
            goal_args = [x.get_args() for x in goal_atoms]
            mem = self.mem
            # get the world
            world = self.mem.myMidca.midca.world
            # initialize the resources for goal transform
            goaltransform.resources(mem)
            transformed_goals = []
            goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
            for i in range(0, len(goal_atoms)):
                trans_goal = goaltransform.choose(mem, goal_atoms[i])
                transformed_goals.append(trans_goal)
                world.cltree['checked'] = []

            # do removal first
            for g in goal_check:
                goalGraph.remove(g)

            if verbose >= 2:
                # now display success statement
                print("Removed from the goal graph the previous goal:")
                for g in goal_check:
                    print(("    " + str(g)))

            # now display transformed goal
            if verbose >= 2:
                print("Goal has been transformed to:")
                for g in transformed_goals:
                    print(("    " + str(g)))

            # now insert the new goal
            for g in transformed_goals:
                goalGraph.insert(g)

            self.mem.set(self.mem.CURRENT_GOALS, transformed_goals)
            if verbose >= 2:
                print("Transformed goal has been inserted into goal graph.")

            return True
            # goals.Goal(*['A_','B_'], predicate = 'stable-on'),
            #       goals.Goal(*['C_','A_'], predicate = 'stable-on'),
            #       goals.Goal(*['D_','C_'], predicate = 'stable-on')

            # and go top down
            # first, get the heights
