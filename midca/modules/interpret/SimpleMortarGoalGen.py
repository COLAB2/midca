from midca import goals, base
from midca import midcatime
from ._goalgen import tf_fire
from ._goalgen import tf_3_scen
from midca.domains.logistics import deliverstate
from midca.domains.blocksworld import blockstate
from midca.worldsim import stateread
import copy,csv
import random
from midca.modules.monitors import Monitor

class SimpleMortarGoalGen(base.BaseModule):
    '''
    MIDCA module that cycles through goals for the agent to achieve.
    '''

    curr_goal_index = 0

    curr_goal_sets = [
                  [goals.Goal(*['A_','B_'], predicate = 'stable-on'),
                   goals.Goal(*['C_','A_'], predicate = 'stable-on'),
                   goals.Goal(*['D_','C_'], predicate = 'stable-on')],
                  [goals.Goal(*['D_','B_'], predicate = 'stable-on'),
                   goals.Goal(*['B_','A_'], predicate = 'stable-on'),
                   goals.Goal(*['A_','C_'], predicate = 'stable-on')]]

    # starting state: on(D,B), on(B,A), ontable(A) ontable(C)
    # first goal: on(C,B)
    # second goal

    def next_goal(self):
        # get the next goal
        curr_goal_set = self.curr_goal_sets[self.curr_goal_index]
        # update index for next time around
        if self.curr_goal_index == len(self.curr_goal_sets)-1:
            self.curr_goal_index = 0
        else:
            self.curr_goal_index+=1
        return curr_goal_set

    def run(self, cycle, verbose=2):
        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)

        # first, check to see if we need a new goal, and only then insert a new one
        if len(self.mem.get(self.mem.GOAL_GRAPH).getAllGoals()) == 0:
            # get the next goal
            goal_set = self.next_goal()
            # insert that goal
            for g in goal_set:
                self.mem.get(self.mem.GOAL_GRAPH).insert(g)
            # update trace
            if trace:
                trace.add_data("NEXT GOAL(s)", goal_set)
                trace.add_data("GOAL GRAPH", copy.deepcopy(self.mem.GOAL_GRAPH))
        else:
            if trace:
                trace.add_data("NEXT GOAL", 'goals not empty; no goal chosen')
                trace.add_data("GOAL GRAPH", copy.deepcopy(self.mem.GOAL_GRAPH))

class SimpleMortarGoalGen_Restaurant(base.BaseModule):
    '''
    MIDCA module that cycles through goals for the agent to achieve.
    '''

    curr_goal_index = 0

    order = 0

    curr_goal_sets = []

    def __init__(self,stateFile,state_str,Money):
        # these are the default state files that are initialized
        self.stateFile = stateFile
        self.state_str = state_str
        self.money = Money

    def remove_bunch_atoms(self,atoms):
        # This function removes all the atoms in the world,follows recursion
        if(len(atoms) == 0):
            return
        if(self.world.atom_true(list(atoms)[len(atoms) - 1])):
            self.world.remove_atom(list(atoms)[len(atoms) - 1])
            if(len(atoms) == 0):
                return
            else:
                atoms.pop()
            self.remove_bunch_atoms(atoms)

    def create_current_goal_sets(self,count_goals,persons_names,dishes_names):
        # generate random sets of goals and append it to the variable curr_goal_sets
        # create "count_goals" number of goals and append it to self.curr_goal_sets
        # get random goals for each person
        # get random no:of dishes ordered by a person

        for i in range(count_goals):
            random.shuffle(dishes_names)
            count_dishes = random.randint(1, len(dishes_names)-1)
            for j in range(count_dishes):
                dish_choice = dishes_names[j]
                person = persons_names[i]
                random_goal = goals.Goal(*[person,dish_choice], predicate = 'order_serve')
                self.curr_goal_sets.append(random_goal)


    def create_random_goals(self):
        # This function creates random set's of goals for different persons and dishes
        # Get the objects of type dishes
        dishes_names = self.world.get_objects_names_by_type("DISH")
        persons_names = self.world.get_objects_names_by_type("PERSON")
        # shuffle the dishes_names and persons_names randomly for random goal generation
        random.shuffle(dishes_names)
        random.shuffle(persons_names)
        # get the no:of random goals to be generated based no:of persons arriving to the restaurant
        count_goals = random.randint(2, len(persons_names)-1)
        # add random goals into self.curr_goal_sets
        self.create_current_goal_sets(count_goals,persons_names,dishes_names)

    def save_30_goal_sets(self):
        # this function is to save the random 30 problem sets in the form of a .pkl file
        a = []
        for i in range(30):
            self.create_random_goals()
            a.append(copy.deepcopy(self.curr_goal_sets))
            self.curr_goal_sets[:]  = []
        with open('30_problem_set_restaurant.pickle', 'wb') as handle:
            pickle.dump(a, handle)

    def select_random_goals_from_30_sets(self):
        # this function is to select each problem from the problem set
        with open('30_problem_set_restaurant.pickle', 'rb') as handle:
            a = pickle.load(handle)
        self.curr_goal_sets = a[self.curr_goal_index]
        self.curr_goal_index = self.curr_goal_index + 1

    def get_problem_set(self):
        # get the persons name
        persons = {}
        for each in self.curr_goal_sets:
            if each.args[0] in persons:
                persons[each.args[0]].append(each.args[1])
            else:
                persons[each.args[0]] = [each.args[1]]
        return persons

    def set_memory_to_none(self):
        '''
        set memory variables to none
        '''
        self.mem.set(self.mem.SELECTED_ORDERS, None)
        self.mem.set(self.mem.COMPLETED_ORDERS, None)
        self.mem.set(self.mem.EXPECTED_SCORE, None)
        self.mem.set(self.mem.ACTUAL_SCORE, None)
        self.mem.set(self.mem.EXPECTED_COST, None)
        self.mem.set(self.mem.ACTUAL_COST, None)

    def write_memory_to_file(self):
        # write the memory variables into file
        myfile  = open('evaluation.csv', "a")
        writer = csv.writer(myfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_ALL)
        if not self.mem.get(self.mem.SELECTED_ORDERS) :
            data = ["S.NO" , "PROBLEM SET" , "SELECTED ORDERS" , "EXPECTED SCORES" ,
                    "ACTUAL SCORES" , "EXPECTED COST", "ACTUAL COST", "EXPECTED(P/T)",
                    "ACTUAL(P/T) " , "COMPLETED ORDERS"
                   ]
            writer.writerow(data)
        else:
            self.order = self.order + 1
            sno = str(self.order)
            selected_orders = str(self.mem.get(self.mem.SELECTED_ORDERS))
            completed_orders = str(self.mem.get(self.mem.COMPLETED_ORDERS))
            problem_set = str(self.get_problem_set())
            expected_scores = self.mem.get(self.mem.EXPECTED_SCORE)
            actual_scores = self.mem.get(self.mem.ACTUAL_SCORE)
            expected_cost = self.mem.get(self.mem.EXPECTED_COST)
            actual_cost = self.mem.get(self.mem.ACTUAL_COST)
            if expected_cost:
                exp_p_t = float(expected_scores)/float(expected_cost)
            else:
                exp_p_t = None
            if actual_cost:
                act_p_t = float(actual_scores)/float(actual_cost)
            else:
                act_p_t = None
            data = [sno, problem_set, selected_orders, str(expected_scores),
                    str(actual_scores), str(expected_cost), str(actual_cost),
                    str(exp_p_t), str(act_p_t), completed_orders
                   ]
            writer.writerow(data)
            self.set_memory_to_none()




    def next_goal(self):
        # empty the previous goals and set the money to default in memory
        self.mem.set(self.mem.MONEY,self.money)
        # remove all the atoms in the world
        atoms = self.world.get_atoms()
        self.remove_bunch_atoms(atoms)
        # whenever we get a new goal, first initialize the world to it's default state
        stateread.apply_state_file(self.world, self.stateFile)
        stateread.apply_state_str(self.world, self.state_str)
        self.mem.add(self.mem.STATES, copy.deepcopy(self.world))
        # Create the random set of goals with random dishes and persons
        self.curr_goal_sets[:] = []
        self.create_random_goals()
        #self.save_30_goal_sets()
        #self.write_memory_to_file()
        #self.select_random_goals_from_30_sets()
        # print the generated goals
        print("")
        for each_goal in self.curr_goal_sets:
            print (each_goal)
        print("")
        # return the generated goals
        return self.curr_goal_sets

    def init(self, world, mem):
        base.BaseModule.init(self, mem)
        self.world = world

    def run(self, cycle, verbose=2):
        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)

        # first, check to see if we need a new goal, and only then insert a new one
        if len(self.mem.get(self.mem.GOAL_GRAPH).getAllGoals()) == 0:
            # get the next goal set
            goal_set = self.next_goal()
            # insert that goal
            for g in goal_set:
                self.mem.get(self.mem.GOAL_GRAPH).insert(g)
            # update trace
            trace.add_data("NEXT GOAL(s)", goal_set)
            trace.add_data("GOAL GRAPH", copy.deepcopy(self.mem.GOAL_GRAPH))
        else:
            trace.add_data("NEXT GOAL", 'goals not empty; no goal chosen')
            trace.add_data("GOAL GRAPH", copy.deepcopy(self.mem.GOAL_GRAPH))


class SimpleMortarGoalGen_construction(base.BaseModule):
    '''
    MIDCA module that cycles through goals for the agent to achieve.
    Generates the building goals(towers) to acheive
    '''
    initial_world = "__initial state"
    selected_goals = []
    count = 0

    curr_goal_sets = []


    def remove_bunch_atoms(self,atoms):
        '''
        input : atoms (state) of the world
        functionality : removes the atoms form the world
        output : none
        '''

        if(len(atoms) == 0):
            return
        if(self.world.atom_true(list(atoms)[len(atoms) - 1])):
            self.world.remove_atom(list(atoms)[len(atoms) - 1])
            if(len(atoms) == 0):
                return
            else:
                atoms.pop()
            self.remove_bunch_atoms(atoms)

    def __init__(self,stateFile,state_str,T):
        '''
        input : stateFile (contains states)
                state_str( contains the mortar atoms)
                Time ( starting time)

        functionality : initialization
        output : class variables
        '''
        self.stateFile = stateFile
        self.state_str = state_str
        self.Time = T

    def find_sum_natural_numbers_nearest_number(self, number):
        '''
        input : random number
        functionality :  find the nearest natural number
        output : return the number
        '''
        for i in range(0,number):
            if (i*(i+1))/2 > number:
                return i

    def build_current_goal_sets(self , objs):
        '''
        input : generate the goals
        functionality : generate the building goals in the oreder of 1,2 -- nearest natural number
        output: update self.curr_goal_sets with building goals
        '''
        number = self.find_sum_natural_numbers_nearest_number(len(objs))
        index = 0
        for goals_number in range(1,number):
            each_goal_set = []
            for i in range(0 , goals_number):
                if i == 0:
                    each_goal_set.append(goals.Goal(*[objs[index]], predicate = 'on-table'))
                    index = index + 1
                else:
                    each_goal_set.append(goals.Goal(*[objs[index],objs[index-1]], predicate = 'stable-on'))
                    index = index + 1
            self.curr_goal_sets.append(each_goal_set)

    def build_current_goal_sets_same(self, objs):
        '''
        input : generate the goals
        functionality : generate the building goals,
                         might be same number of goals in towers as well
        output: update self.curr_goal_sets with building goals
        '''
        number = self.find_sum_natural_numbers_nearest_number(len(objs))
        index = 0
        for goals_number in range(1,number-1):
            each_goal_set = []
            goals_no = random.randint(1, number-1)
            if (index > (len(objs) - 1)):
                break
            for i in range(0 , goals_no):
                if (index > (len(objs) - 1)):
                    break

                if i == 0:
                    each_goal_set.append(goals.Goal(*[objs[index]], predicate = 'on-table'))
                    index = index + 1
                else:
                    each_goal_set.append(goals.Goal(*[objs[index],objs[index-1]], predicate = 'stable-on'))
                    index = index + 1
            self.curr_goal_sets.append(each_goal_set)


    def save_30_buildings(self):
        '''
        save the 30 building goals into the 30_problem_set_ijcai.pickle file
        '''
        a = {}
        for k in range(0,30):
            self.curr_goal_sets[:] = []
            select_buildings = 0
            index = []
            objs_names = self.world.get_objects_names_by_type("BLOCK")
            # remove the object table since we need only blocks
            objs_names.remove("table")
            random.shuffle(objs_names)
            # builds the gaol sets
            #self.build_current_goal_sets(objs_names)
            self.build_current_goal_sets_same(objs_names)
            atoms = self.world.get_atoms()
            self.remove_bunch_atoms(atoms)
            stateread.apply_state_file(self.world, self.stateFile)
            stateread.apply_state_str(self.world, self.state_str)
            self.mem.add(self.mem.STATES, copy.deepcopy(self.world))
            self.mem.set(self.mem.TIME_CONSTRUCTION, self.Time)
            #print(self.world)
            # empty the selected goals list
            del self.selected_goals[:]
            # generate some random goals through random function on current goal set
            # this random function is for no:of buildings
            select_buildings = random.randint(2, len(self.curr_goal_sets)-1)
            print(("NO.OF BUILDINGS TO CONSTRUCT: " + str(select_buildings)))
            # this is for the random indexes , that should be taken from the variable self.curr_goal_sets
            # compute the index list with in the range of 0 and no:of buildings
            index = random.sample(list(range(0, len(self.curr_goal_sets))), select_buildings)
            print("THE BUILDINGS ARE: ")
            print("[")
            for i in index:
            # since self.curr_goal_sets is in a structure of list in a list
            # we should iterate through the list completely
                print(("Tower " + self.curr_goal_sets[i][0].args[0]), end=' ')
                if(len(self.curr_goal_sets[i]) == 1):
                    print(("(" + str(len(self.curr_goal_sets[i])) + " Block)"))
                else:
                    print(("(" + str(len(self.curr_goal_sets[i])) + " Blocks)"))
                for j in self.curr_goal_sets[i]:
                    #print(j)
                    self.selected_goals.append(j)
                #print("")
            print("]")
            print("")
            random.shuffle(index)
            print(index)
            a[k] =[self.curr_goal_sets , select_buildings , copy.deepcopy(index)]
            with open('30_problem_set_ijcai.pickle', 'wb') as handle:
                pickle.dump(a, handle)

    def next_goal_30(self):
        '''
        Generate the goals from the 30 sets
        '''
        actual_time = self.mem.get(self.mem.ACTUAL_TIME_CONSTRUCTION)
        expected_time = self.mem.get(self.mem.EXPECTED_TIME_CONSTRUCTION)
        selected_buildings = self.mem.get(self.mem.SELECTED_BUILDING_LIST)
        complete_buildings = self.mem.get(self.mem.COMPLETE_BUILDING_LIST)
        executed_buildings = self.mem.get(self.mem.EXECUTED_BUILDING_LIST)
        actual_scores = self.mem.get(self.mem.ACTUAL_SCORES)
        P = self.mem.get(self.mem.P)
        t = self.mem.get(self.mem.t)
        P = str(P)
        t = str(t)
        P = P.replace("[...]" , "")
        t = t.replace("[...]" , "")
        if not actual_scores:
            actual_scores = 0


        if executed_buildings:
            expected_scores = sum(selected_buildings)
        else:
            expected_scores = 0
        if complete_buildings:
            if expected_time:
                expected_p_t = float(expected_scores/expected_time)
            else:
                e_t = None
                e_p_t = None
                e_s = None

            if actual_time:
                actual_p_t = float(actual_scores/actual_time)
            else:
                a_t = None
                a_s = None
                a_p_t = None

            myfile  = open('evaluation.csv', "a")
            writer = csv.writer(myfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_ALL)
            with open("evaluation.csv", "a") as myfile:
                if expected_time:
                    e_t = round(expected_time,2)
                    e_p_t = round(expected_p_t , 2)
                if actual_time:
                    a_t = round(actual_time,2)
                    a_p_t = round(actual_p_t , 2)
                if expected_scores:
                    e_s = round(expected_scores,2)
                if expected_scores == 0:
                    e_s = 0
                if actual_scores:
                    a_s = round(actual_scores , 2 )

                if actual_scores == 0:
                    a_s = 0
                e_b = executed_buildings
                data = [self.count , complete_buildings , selected_buildings , e_t , a_t ,
                       e_s , a_s , e_p_t , a_p_t , e_b, str(P), str(t)
                       ]
                writer.writerow(data)
            self.mem.set(self.mem.ACTUAL_TIME_CONSTRUCTION , None)
            self.mem.set(self.mem.EXPECTED_TIME_CONSTRUCTION , None)
            self.mem.set(self.mem.SELECTED_BUILDING_LIST , None)
            self.mem.set(self.mem.COMPLETE_BUILDING_LIST , None)
            self.mem.set(self.mem.EXECUTED_BUILDING_LIST, None)
            self.mem.set(self.mem.ACTUAL_SCORES, None)
            self.mem.set(self.mem.P, None)
            self.mem.set(self.mem.t, None)

        else:
            myfile  = open('evaluation.csv', "a")
            writer = csv.writer(myfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_ALL)
            data = ["S.NO" , "PROBLEM SET" , "SELECTED TOWERS" , "EXPECTED TIME" ,
                    "ACTUAL TIME", "EXPECTED SCORES" , "ACTUAL SCORES" , "EXPECTED(P/T)" ,
                    "ACTUAL(P/T) " , "CONSTRUCTED BUILDINGS", "P" , "t"
                   ]
            writer.writerow(data)

        self.curr_goal_sets[:] = []
        objs_names = self.world.get_objects_names_by_type("BLOCK")
        # remove the object table since we need only blocks
        objs_names.remove("table")
        random.shuffle(objs_names)
        # builds the gaol sets
        self.build_current_goal_sets(objs_names)
        atoms = self.world.get_atoms()
        self.remove_bunch_atoms(atoms)
        stateread.apply_state_file(self.world, self.stateFile)
        stateread.apply_state_str(self.world, self.state_str)
        self.mem.add(self.mem.STATES, copy.deepcopy(self.world))
        self.mem.set(self.mem.TIME_CONSTRUCTION, self.Time)
        #print(self.world)
        # empty the selected goals list
        del self.selected_goals[:]

        with open('30_problem_set_ijcai.pickle', 'rb') as handle:
            a = pickle.load(handle)
        b = a[self.count]
        self.count = self.count + 1
        select_buildings = b[1]
        print(("NO.OF BUILDINGS TO CONSTRUCT: " + str(select_buildings)))
        # this is for the random indexes , that should be taken from the variable self.curr_goal_sets
        # compute the index list with in the range of 0 and no:of buildings
        index = b[2]
        self.curr_goal_sets = b[0]
        print("THE BUILDINGS ARE: ")
        print("[")
        for i in index:
            # since self.curr_goal_sets is in a structure of list in a list
            # we should iterate through the list completely
            print(("Tower " + self.curr_goal_sets[i][0].args[0]), end=' ')
            if(len(self.curr_goal_sets[i]) == 1):
                print(("(" + str(len(self.curr_goal_sets[i])) + " Block)"))
            else:
                print(("(" + str(len(self.curr_goal_sets[i])) + " Blocks)"))
            for j in self.curr_goal_sets[i]:
                #print(j)
                self.selected_goals.append(j)
            #print("")
        print("]")
        print("")

        return self.selected_goals

    def next_goal_30_goal_transformations(self,write_to_file = False):
        selected_buildings = self.mem.get(self.mem.SELECTED_BUILDING_LIST)
        executed_buildings = self.mem.get(self.mem.EXECUTED_BUILDING_LIST)
        actual_scores = self.mem.get(self.mem.ACTUAL_SCORES)

        if executed_buildings :
            myfile  = open('evaluation.csv', "a")
            writer = csv.writer(myfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_ALL)
            with open("evaluation.csv", "a") as myfile:
                data = [(self.count),selected_buildings, actual_scores,executed_buildings]
                writer.writerow(data)
            self.mem.set(self.mem.SELECTED_BUILDING_LIST, None)
            self.mem.set(self.mem.EXECUTED_BUILDING_LIST, None)
            self.mem.set(self.mem.ACTUAL_SCORES, None)
        else:
            myfile  = open('evaluation.csv', "a")
            writer = csv.writer(myfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_ALL)
            data = ["S.NO" , "PROBLEM SET", "ACTUAL SCORES" , "CONSTRUCTED BUILDINGS" ]
            writer.writerow(data)

        self.curr_goal_sets[:] = []
        objs_names = self.world.get_objects_names_by_type("BLOCK")
        # remove the object table since we need only blocks
        objs_names.remove("table")
        random.shuffle(objs_names)
        # builds the gaol sets
        self.build_current_goal_sets(objs_names)
        atoms = self.world.get_atoms()
        self.remove_bunch_atoms(atoms)
        stateread.apply_state_file(self.world, self.stateFile)
        stateread.apply_state_str(self.world, self.state_str)
        self.mem.add(self.mem.STATES, copy.deepcopy(self.world))
        self.mem.set(self.mem.TIME_CONSTRUCTION, self.Time)
        print((self.world))
        # empty the selected goals list
        del self.selected_goals[:]

        with open('30_problem_set.pickle', 'rb') as handle:
            a = pickle.load(handle)
        b = a[self.count]
        self.count = self.count + 1
        select_buildings = b[1]
        print(("NO.OF BUILDINGS TO CONSTRUCT: " + str(select_buildings)))
        # this is for the random indexes , that should be taken from the variable self.curr_goal_sets
        # compute the index list with in the range of 0 and no:of buildings
        index = b[2]
        self.curr_goal_sets = b[0]
        for each in self.curr_goal_sets:
            for goal in each:
                if goal["predicate"] == "on":
                    goal["predicate"] = "stable-on"
        print("THE BUILDINGS ARE: ")
        print("[")
        for i in index:
            # since self.curr_goal_sets is in a structure of list in a list
            # we should iterate through the list completely
            print(("Tower " + self.curr_goal_sets[i][0].args[0]), end=' ')
            if(len(self.curr_goal_sets[i]) == 1):
                print(("(" + str(len(self.curr_goal_sets[i])) + " Block)"))
            else:
                print(("(" + str(len(self.curr_goal_sets[i])) + " Blocks)"))
            for j in self.curr_goal_sets[i]:
                #print(j)
                self.selected_goals.append(j)
            #print("")
        print("]")
        print("")

        return self.selected_goals




    def next_goal(self,write_to_file = False):
        actual_time = self.mem.get(self.mem.ACTUAL_TIME_CONSTRUCTION)
        expected_time = self.mem.get(self.mem.EXPECTED_TIME_CONSTRUCTION)
        selected_buildings = self.mem.get(self.mem.SELECTED_BUILDING_LIST)
        complete_buildings = self.mem.get(self.mem.COMPLETE_BUILDING_LIST)
        executed_buildings = self.mem.get(self.mem.EXECUTED_BUILDING_LIST)
        actual_scores = self.mem.get(self.mem.ACTUAL_SCORES)
        if executed_buildings:
            expected_scores = sum(executed_buildings)
        else:
            expected_scores = 0
        if actual_time and expected_time:
            expected_p_t = float(expected_scores/expected_time[0])
            actual_p_t = float(actual_scores/actual_time[0])

            if write_to_file :
                with open("evaluation.txt", "a") as myfile:

                    myfile.write("%-5s %-5s %-5s %-5s "
                                 %( str(complete_buildings),
                                    str(selected_buildings) ,
                                    str(expected_time),
                                    str(actual_time)
                                  )
                                )
                    myfile.write("%-5s %-5s %-5s %-5s %-5s "
                                 % (  str([expected_scores]) ,
                                      str([actual_scores]) ,
                                      str([expected_p_t]) ,
                                      str([actual_p_t]) ,
                                      str(executed_buildings)
                                    )
                                )
                    myfile.write("\n")

            self.mem.set(self.mem.ACTUAL_TIME_CONSTRUCTION , None)
            self.mem.set(self.mem.EXPECTED_TIME_CONSTRUCTION , None)
            self.mem.set(self.mem.SELECTED_BUILDING_LIST , None)
            self.mem.set(self.mem.COMPLETE_BUILDING_LIST , None)
            self.mem.set(self.mem.EXECUTED_BUILDING_LIST, None)
            self.mem.set(self.mem.ACTUAL_SCORES, None)

        #print(self.mem.get(self.mem.STATES)[-1])
        #print(self.initial_world)
        # initiate the world
        #predicateworld.asqiiDisplay(world)
        #self.world =  self.initial_world.copy()
        #print("Modified world")
        #print(self.world)
        #base.MIDCA.update_world(self,self.initial_world.copy())
        #self.mem.add(self.mem.STATES, self.initial_world.copy())
        #print(self.mem.get(self.mem.STATES)[-1]
        #atoms = self.initial_world.get_atoms()
        #atoms1 = self.initial_world.get_atoms()
        #for atom in atoms1:
        #       self.world.add_atom(atom)
        # get all the object names of type block
        self.curr_goal_sets[:] = []
        objs_names = self.world.get_objects_names_by_type("BLOCK")
        # remove the object table since we need only blocks
        objs_names.remove("table")
        random.shuffle(objs_names)
        # builds the gaol sets
        #self.build_current_goal_sets(objs_names)
        self.build_current_goal_sets_same(objs_names)
        atoms = self.world.get_atoms()
        self.remove_bunch_atoms(atoms)
        stateread.apply_state_file(self.world, self.stateFile)
        stateread.apply_state_str(self.world, self.state_str)
        self.mem.add(self.mem.STATES, copy.deepcopy(self.world))
        self.mem.set(self.mem.TIME_CONSTRUCTION, self.Time)
        #print(self.world)
        # empty the selected goals list
        del self.selected_goals[:]
        # generate some random goals through random function on current goal set
        # this random function is for no:of buildings
        select_buildings = random.randint(2, len(self.curr_goal_sets)-1)
        print(("NO.OF BUILDINGS TO CONSTRUCT: " + str(select_buildings)))
        # this is for the random indexes , that should be taken from the variable self.curr_goal_sets
        # compute the index list with in the range of 0 and no:of buildings
        index = random.sample(list(range(0, len(self.curr_goal_sets))), select_buildings)
        print("THE BUILDINGS ARE: ")
        print("[")
        for i in index:
            # since self.curr_goal_sets is in a structure of list in a list
            # we should iterate through the list completely
            print(("Tower " + self.curr_goal_sets[i][0].args[0]), end=' ')
            if(len(self.curr_goal_sets[i]) == 1):
                print(("(" + str(len(self.curr_goal_sets[i])) + " Block)"))
            else:
                print(("(" + str(len(self.curr_goal_sets[i])) + " Blocks)"))
            for j in self.curr_goal_sets[i]:
                #print(j)
                self.selected_goals.append(j)
        print("]")
        print("")

        return self.selected_goals

    def init(self, world, mem):
        base.BaseModule.init(self, mem)
        self.world = world


    def run(self, cycle, verbose=2):
        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)

        # first, check to see if we need a new goal, and only then insert a new one
        if len(self.mem.get(self.mem.GOAL_GRAPH).getAllGoals()) == 0:
            # get the next goal
            #self.save_30_buildings()
            goal_set = self.next_goal()
            # insert that goal
            for g in goal_set:
                self.mem.get(self.mem.GOAL_GRAPH).insert(g)
            # update trace
            trace.add_data("NEXT GOAL(s)", goal_set)
            trace.add_data("GOAL GRAPH", copy.deepcopy(self.mem.GOAL_GRAPH))
            trace.add_data("WORLD",copy.deepcopy(self.world))
        else:
            trace.add_data("NEXT GOAL", 'goals not empty; no goal chosen')
            trace.add_data("GOAL GRAPH", copy.deepcopy(self.mem.GOAL_GRAPH))
