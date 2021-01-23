from midca import base,midcatime
import copy,itertools,operator
import random
import time as ctime
import numpy as np
import math


class SimpleIntend(base.BaseModule):

    def run(self, cycle, verbose = 2):
        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("GOALGRAPH",copy.deepcopy(self.mem.GOAL_GRAPH))

        goalGraph = self.mem.get(self.mem.GOAL_GRAPH)

        if not goalGraph:
            if verbose >= 1:
                print "Goal graph not initialized. Intend will do nothing."
            return
        # get all the goals from the root of the goal graph
        goals = goalGraph.getUnrestrictedGoals()

        if not goals:
            if verbose >= 1:
                print "No Goals in Goal graph. Intend will do nothing."
            return

        # take the first goal
        goals = [goals[0]]
        # add it to the current goal in memory

        # current goals as a stack
        if self.mem.get(self.mem.CURRENT_GOALS) :
            current_goals = self.mem.get(self.mem.CURRENT_GOALS)
            if not current_goals[-1] == goals:
                current_goals.append(goals)
                self.mem.set(self.mem.CURRENT_GOALS, current_goals)
            else:
                goals = []
        else:
            self.mem.set(self.mem.CURRENT_GOALS, [goals])

        if trace:
            trace.add_data("GOALS",goals)

        if not goals:
            if verbose >= 2:
                print "No goals selected."
        else:
            if verbose >= 2:
                print "Selecting goal(s):",
                for goal in goals:
                    print goal,
                print

class WarehouseIntend(base.BaseModule):

    def run(self, cycle, verbose = 2):
        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("GOALGRAPH",copy.deepcopy(self.mem.GOAL_GRAPH))

        goalGraph = self.mem.get(self.mem.GOAL_GRAPH)

        if not goalGraph:
            if verbose >= 1:
                print "Goal graph not initialized. Intend will do nothing."
            return
        goals = goalGraph.getUnrestrictedGoals()
        goals_selected = []
        # special code for NBeacons, need to change for later
        exists_free_goal = False
        free_goal = None
        for g in goals:
            if 'free' in str(g):
                exists_free_goal = True
                free_goal = g

        # activation goals: just pick one

        if free_goal:
            goals_selected = [free_goal]
            self.mem.set(self.mem.CURRENT_GOALS,[goals_selected] )
        else:
            if len(goals) > 1:
                # accomplish one of the warehouse packages
                goals_selected = []
                goals_selected_warehouse = goals[0].args[2]
                for g in goals:
                    if g.args[2] == goals_selected_warehouse:
                        goals_selected.append(g)

                self.mem.set(self.mem.CURRENT_GOALS,[goals_selected] )
            else:
                goals_selected = goals
                self.mem.set(self.mem.CURRENT_GOALS, [goals_selected])
        if trace:
            trace.add_data("GOALS",goals)

        if not goals:
            if verbose >= 2:
                print "No goals selected."
        else:
            if verbose >= 2:
                print "Selecting goal(s):",
                for goal in goals_selected:
                    print goal,
                print

class SimpleIntend_Restaurant(base.BaseModule):

    cost_dishes = {} # cost of dishes
    #time_dishes = {} # time taken to cook dishes
    selected_goals = []
    actual_cost = 0 # actual cost
    actual_score = 0 # actual score
    variation = 0.5
    # "selection_method" , "fifo_method"
    method = "FIFO_method" # this denotes whether it is a selection method or FIFO
                                # for FIFO

    def get_cost(self,goals):
        '''
        input : goals
        output : doesnt return anything but updates the cost_dishes and time_dishes variables
        from the world get the atoms and from the atoms
        get the "costofdish" and "timetomakedish" predicates
        '''
        world = self.mem.get(self.mem.STATES)[-1]
        for atom in world.atoms:
            if atom.predicate.name == "costofdish":
                self.cost_dishes[atom.args[0].name] = int(atom.args[1].name)

    def persons_combination(self,goals):
        '''
        input: goals
        output: return the dictionay file that contains all the combinations of persons with
                values of money less than that of in memory
        get time from the memory
        compute cost for each person from the goals
        make all the combinations of the persons interms of tuples along with the cost
        and save it in dictionary
        return the dictionary
        '''
        compute_person_combinations = {}
        compute_person_cost = {}
        money = self.mem.get(self.mem.MONEY)
        # goal.args[0] = name of person
        # goal.args[1] = name of the dish
        # check if the person is already present in the dictionary
        # if already present add cost of dish to the previous cost
        # else just assign cost of dish
        for goal in goals:
            if goal.args[0] in compute_person_cost:
                sum_cost = compute_person_cost[goal.args[0]] + self.cost_dishes[goal.args[1]]
            else:
                sum_cost = self.cost_dishes[goal.args[1]]
            compute_person_cost[goal.args[0]] = sum_cost
        #print("compute_person_cost " + str(compute_person_cost))
        # get the combinations of the persons with less than Money from memory
        # iter.combinations is a predefined method which results in the combinations of number specefied
        # save it in the compute_person_combinations dictionary only if it got the total cost to be less than Money in memory
        for i in range(1,len(compute_person_cost)+1):
            for each_tuple in itertools.combinations(compute_person_cost, i):
                sum_cost = 0
                for person in each_tuple:
                    sum_cost = sum_cost + compute_person_cost[person]
                if sum_cost <= money:
                    compute_person_combinations[each_tuple] = sum_cost
        #print("compute_person_combinations " + str(compute_person_combinations))
        return compute_person_combinations

    def best_cost_combination(self,goals,combinations):
        '''
        input : same satisfiability(score) combination of type list
        output : a combination which has more cost
        '''
        best_combination = {}
        total_cost = 0
        cost_compute = [] # list of costs for each person in best combinations
        for each_combination in combinations:
            cost_compute[:] = []
            for each in each_combination:
                cost = 0
                for goal in goals:
                    if each in goal.args:
                        cost = cost + self.cost_dishes[goal.args[1]]
                cost_compute.append(cost)
            if total_cost < sum(cost_compute):
                best_combination = {}
                best_combination[each_combination] = copy.deepcopy(cost_compute)
                total_cost = sum(cost_compute)
        return best_combination

    def get_goals_p_t(self,combination,goals):
        '''
        input: goals and the combination with maximum cost
        output: dictionary of goals with p/t scores
        '''
        goal_combination = {}
        select_goals = []
        for each_combination in combination:
            for id,each in enumerate(each_combination):
                select_goals[:]= []
                for goal in goals:
                    if each in goal.args:
                        select_goals.append(goal)
                select_goals_tuple = tuple(select_goals)
                goal_combination[select_goals_tuple] = [len(select_goals_tuple) ,combination[each_combination][id]]
        return goal_combination



    def compute_time_expectation(self,goals):
        '''
        input : goals
        output: expected time
        iterate through the goals, calculate the total time taken for the dishes
        '''
        time = 0
        for goal in goals:
            time = time + self.time_dishes[goal.args[1]]
        return time

    def compute_cost_expectation(self,goals):
        '''
        input : goals
        output: expected cost
        iterate through the goals, calculate the total time taken for the dishes
        '''
        cost = 0
        for goal in goals:
            cost = cost + self.cost_dishes[goal.args[1]]
        return cost

    def compute_score_expectation(self,goals):
        '''
        input : goals
        output: expected score (satisfaction)
        '''
        return len(goals)

    def write_expected_to_memory(self,expected_score,expected_cost):
        '''
        input : expected_time , expected_score, expected_cost
        output : place all these into memory
        '''
        if not self.mem.get(self.mem.EXPECTED_COST):
            #print(expected_score)
            self.mem.set(self.mem.EXPECTED_SCORE, expected_score)
            self.mem.set(self.mem.EXPECTED_COST, expected_cost)

    def write_selected_to_memory(self,selected_goals):
        '''
        input : selected goals
        output : write the orders to the memory
        iterate through the selected goals, get each tuple and append it to mem variable
        '''
        selected_orders = self.mem.get(self.mem.SELECTED_ORDERS)
        orders = []
        if not selected_orders:
            for goal_set in selected_goals:
                for goal in goal_set:
                    orders.append(goal.args[0])
                    break
            self.mem.set(self.mem.SELECTED_ORDERS,orders)


    def calculate_combinations_best_scores(self,combination,goals):
        '''
        input : combinations of the persons with less than the money in memory
        output : return a list of combination or combinations with highest satisfybility
        '''
        person_satisfybility = {}
        combination_with_satisfyability = {}
        same_satisfaction_combinations = []
        # for each goal with a person the satisfybility increases by 1
        for goal in goals:
            if goal.args[0] in person_satisfybility:
                person_satisfybility[goal.args[0]] += 1
            else:
                person_satisfybility[goal.args[0]] = 0
        # compute the satisfybility for combinations
        for persons in combination:
            satisfaction = 0
            for person in persons:
                satisfaction = satisfaction + person_satisfybility[person]
            combination_with_satisfyability[persons] = satisfaction
        #print("combination_with_satisfyability" + str(combination_with_satisfyability))
        # get the best satisfaction count
        maximum = 0
        for i in sorted( combination_with_satisfyability, key=combination_with_satisfyability.__getitem__ ,reverse = True):
            if maximum <= combination_with_satisfyability[i]:
                maximum = combination_with_satisfyability[i]
                same_satisfaction_combinations.append(i)
        #print(same_satisfaction_combinations)
        return same_satisfaction_combinations

    def goals_combination(self,persons_combinations,goals):
        '''
        input : persons_combinations (all the combination of persons with Money less than Money in memory)
                goals (given by user)
        output : goals_combinations (opt the combination with greater satisfybility,
                                    order the goals according to the (P/T) ratio
                                    P = satisfaction, T = Cost)
        '''
        same_length_combinations = self.calculate_combinations_best_scores(persons_combinations,goals)
        #print("same_length_combinations" + str(same_length_combinations))
        # get the combination which has best cost
        combination = self.best_cost_combination(goals,same_length_combinations)
        #print("combination" + str(combination))
        # get the goals with p,t values
        goals_combinations = self.get_goals_p_t(combination,goals)
        # sort the goals with p/t
        sorted_goals = {}
        for each in goals_combinations:
            sorted_goals[each] = goals_combinations[each][0]/goals_combinations[each][1]
        selected_goals = sorted(sorted_goals , key=sorted_goals.get ,reverse =True)
        # print the P/T ratio , Expected time and Total expected time for persons
        total_score = 0
        total_cost = 0
        for each in selected_goals :
            for goal in each:
                p = goals_combinations[each][0]
                t = goals_combinations[each][1]
                print("")
                print ("For the Person " + goal.args[0] + ": P/T ratio (" + str(p)+"/"+str(t) + ")  = " + str(float(p)/float(t)))
                expected_score = self.compute_score_expectation(each)
                expected_cost = self.compute_cost_expectation(each)
                total_score = expected_score + total_score
                total_cost = expected_cost + total_cost
                print("\t\t   Expected Score : " + str(expected_score))
                print("\t\t   Expected Cost : " + str(expected_cost))
                break
        print("Expected Total Score : " + str(total_score))
        print("Expected Total Cost : " + str(total_cost))
        print("")
        self.write_expected_to_memory(total_score,total_cost)
        self.write_selected_to_memory(selected_goals)
        self.selected_goals = selected_goals
        return selected_goals



    def select_priority_goals(self,goals):
        '''
        input: goals
        output: return the goals that are selected by the goal selecting method
        goal selecting method : get the combinations of the goals that takes time less than that of time in memory
                                opt the combination with greater satisfybility
                                order the goals according to the (P/T) ratio P = satisfaction, T = Cost
                                return the goals
        '''
        # get the cost and time taken to cook dishes
        self.get_cost(goals)
        #print("cost_dishes " + str(self.cost_dishes))
        # make the combination of persons that usually result in making less than or equal to Time in memory
        persons_combinations = self.persons_combination(goals)
        #print("persons_combinations" + str(persons_combinations))
        # make the combination of goals based on the maximum satisfaction(score)
        goals_combinations = self.goals_combination(persons_combinations,goals)
        for each in goals_combinations:
            if each:
                return list(each)

    def compute_cost_goal(self,goal):
        '''
        output: calculate the cost computed for the goal
        '''
        #print(goal)
        cost_dishes_p = ((self.variation)* (self.cost_dishes[goal.args[1]])) + self.cost_dishes[goal.args[1]]
        cost_dishes_n = (self.cost_dishes[goal.args[1]]) - ((self.variation)*(self.cost_dishes[goal.args[1]]))
        return  random.uniform(cost_dishes_p, cost_dishes_n)

    def print_goals(self,goals):
        for goal in goals:
            print(goal)

    def remove_from_selected_goals(self,goal):
        '''
        input : goal
        output : remove the goal from the selected goals
        '''
        for id,goal_set in enumerate(self.selected_goals):
            for each_goal in goal_set:
                if each_goal.args == goal.args and each_goal["predicate"] == goal["predicate"]:
                    temp_list = list(self.selected_goals[id])
                    #print("temp_list" + str(temp_list))
                    temp_list.remove(each_goal)
                    #print(temp_list)
                    self.selected_goals[id] = tuple(temp_list)
                    return



    def check_goal_in_current_goals(self,goal):
        '''
        input : goal
        output : return True if the goal is in current goals from memory
                 else return false
        '''

        current_goals = self.mem.get(self.mem.CURRENT_GOALS)
        # if there are no current goals return False
        if not current_goals:
            return False
        #print("Current Goals : ")
        #self.print_goals(current_goals)
        for each_goal in current_goals:
            if goal.args == each_goal.args and goal["predicate"] == each_goal["predicate"]:
                return each_goal
        return False

    def get_score(self,goal):
        '''
        input : goal
        output : return the score of the goal
        '''
        score_p = self.variation + 1
        score_n = 1 - self.variation
        return random.uniform(score_p, score_n)

    def write_executed_goal_memory(self,goal):
        '''
        input : goal
        output: get the person name from the goal and append it to memory
        '''
        completed_goals = self.mem.get(self.mem.COMPLETED_ORDERS)
        person_name = goal.args[0]
        if not completed_goals:
            self.mem.set(self.mem.COMPLETED_ORDERS, [person_name])
        else:
            completed_goals.append(person_name)
            self.mem.set(self.mem.COMPLETED_ORDERS, completed_goals)

    def check_cost(self, cost):
        '''
        input : cost
        output : return False if time is less than zero
                 return true

        '''
        cost_comp =self.mem.get(self.mem.MONEY)
        if (cost_comp - cost) < 0:
            self.mem.set(self.mem.MONEY, 0)
            self.selected_goals[:] = []
            return False
        return True

    def calculate_cost_remaining(self):
        '''
        output : return time taken and remove the goals from the selected goals
                 if the goal doesnot exist in current goal
        '''
        world = self.mem.get(self.mem.STATES)[-1]
        goals = copy.deepcopy(self.selected_goals)
        for id,goals_set in enumerate(goals):
            for i,goal in enumerate(goals_set):
                check_goal = self.check_goal_in_current_goals(goal)
                if check_goal:
                    if world.atom_true(world.midcaGoalAsAtom(check_goal)):
                        self.remove_from_selected_goals(goal)
                        cost_taken = self.compute_cost_goal(goal)
                        score = self.get_score(goal)
                        self.actual_cost = self.actual_cost + cost_taken
                        self.actual_score = self.actual_score + score
                        #print("score" + str(score))
                        return cost_taken,score
                    return 0,0
                else:
                    cost_taken = self.compute_cost_goal(goal)
                    score = self.get_score(goal)
                    #print("score" + str(score))
                    if self.check_cost(cost_taken):
                        self.write_executed_goal_memory(goal)
                        self.write_actual_parameters_memory(cost_taken,score)
                        self.actual_cost = 0
                        self.actual_score = 0
                        # without variation
                        if self.variation == 0:
                            self.selected_goals.pop(id)
                        # with variation
                        else:
                            del self.selected_goals
                            self.selected_goals = []
                    return cost_taken,score
                return 0,0

    def write_actual_parameters_memory(self,cost_taken,score):
        '''
        takes the actual parameters and writes them to memory
        '''
        actual_cost = copy.deepcopy(self.actual_cost)
        actual_score = copy.deepcopy(self.actual_score)
        actual_cost = actual_cost + cost_taken
        actual_score = actual_score + score
        if self.mem.get(self.mem.ACTUAL_COST):
            actual_cost = self.mem.get(self.mem.ACTUAL_COST) + actual_cost
            actual_score = self.mem.get(self.mem.ACTUAL_SCORE) + actual_score
        #print(actual_cost)
        #print(actual_score)
        self.mem.set(self.mem.ACTUAL_COST, actual_cost)
        self.mem.set(self.mem.ACTUAL_SCORE, actual_score)

    def get_current_goals(self):
        '''
        output: if there exists any pending goals, place them into current goals
        calculate time remaining.
        '''
        print("")
        print("Not all Current goals achieved ,")
        print("Processing : "),
        self.print_goals(self.mem.get(self.mem.CURRENT_GOALS))
        print("")
        return self.mem.get(self.mem.CURRENT_GOALS)

    def check_existing_goals(self):
        '''
        check if there are current goals, if not check if there are pending goals
        output : if there exists any pending goals, place them into current goals
                calculate time remaining.
        '''
        if not self.selected_goals == [()]:
            #print(self.selected_goals)
            money = self.mem.get(self.mem.MONEY)
            cost_taken,score = self.calculate_cost_remaining()
            if self.check_cost(cost_taken):
                self.mem.set(self.mem.MONEY, (money - cost_taken))
                print("actual_cost" +str(self.actual_cost))
                print("actual_score" + str(self.actual_score))
                print("Amount Spent for previous dish order : " + str(cost_taken))
        if self.mem.get(self.mem.CURRENT_GOALS):
            return self.get_current_goals()
        if not self.selected_goals == [()]:
            for goal_set in self.selected_goals:
                goal = list(goal_set)
                self.mem.set(self.mem.CURRENT_GOALS,goal)
                return self.mem.get(self.mem.CURRENT_GOALS)

    def fifo_method_goals(self,goals):
        '''
        input : goals a list which contains the goals of persons as an element
        output: return the selected goals
        '''
        total_score = 0
        total_cost = 0
        for each in goals :
            for goal in each:
                print ("For the Person " + goal.args[0] )
                expected_score = self.compute_score_expectation(each)
                expected_cost = self.compute_cost_expectation(each)
                total_score = expected_score + total_score
                total_cost = expected_cost + total_cost
                print("\t\t   Expected Score : " + str(expected_score))
                print("\t\t   Expected Cost : " + str(expected_cost))
                break
        print("Expected Total Score : " + str(total_score))
        print("Expected Total Cost : " + str(total_cost))
        print("")
        self.write_expected_to_memory(total_score,total_cost)
        self.write_selected_to_memory(goals)
        self.selected_goals = goals
        return goals

    def selection_method(self,goals):
        '''
        input: goals from the goal graph
        output : returns the selected goals by using selection method
        '''
        # select the goals based on the score,time and cost
        goals = self.select_priority_goals(goals)
        # add it to the current goal in memory
        return goals

    def seperate_goals_fifo(self,goals):
        '''
        input : goals
        output : seperate the goals for a person from the whole goals
        '''
        selected_goals = []
        temp_dict = {}
        temp = []
        for goal in goals:
            if goal.args[0] in temp_dict:
                temp.append(goal)
            else:
                if temp:
                    selected_goals.append(tuple(temp))
                    temp = []
                temp.append(goal)
            temp_dict[goal.args[0]] = goal.args[1]
        if goals:
            selected_goals.append(tuple(temp))

        cost = 0
        temp_goals = copy.deepcopy(selected_goals)
        for id,goal_set in enumerate(temp_goals):
            for goal in goal_set:
                cost = cost + self.cost_dishes[goal.args[1]]
            if cost > self.mem.get(self.mem.MONEY):
                selected_goals = selected_goals[:id]
                return selected_goals


        return selected_goals


    def fifo_selection(self,goals):
        '''
        input: goals from the goal graph
        output : returns the selected goals by using FIFO selection method
        '''
        # get the cost and time taken to cook dishes

        self.get_cost(goals)
        selected_goals = self.seperate_goals_fifo(goals)
        self.fifo_method_goals(selected_goals)
        for each in self.selected_goals:
            if each:
                return list(each)


    def run(self, cycle, verbose = 2):
        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("GOALGRAPH",copy.deepcopy(self.mem.GOAL_GRAPH))

        goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
        goals = None
        if not goalGraph:
            if verbose >= 1:
                print "Goal graph not initialized. Intend will do nothing."
            return

        # if there is no money remaining then intend does nothing
        if self.mem.get(self.mem.MONEY):
            if self.mem.get(self.mem.MONEY) <= 0:
                self.selected_goals[:] = []
                self.mem.set(self.mem.MONEY, 0)
                self.actual_score = 0
                self.actual_cost = 0
                print "Money Insufficient. Intend will do nothing."
                return

        # for selection_method we need to check whether the remaining goals can be done with time
        # remaining
        if self.method == "selection_method":
            money = self.mem.get(self.mem.MONEY)
            cost = 0
            if self.selected_goals:
                for goal in self.selected_goals[0]:
                    cost = cost + self.cost_dishes[goal.args[1]]
                if cost > money:
                    print("Remaining goals for the order " + str(self.selected_goals[0][0].args[0]) + " Cannot be done")
                    self.selected_goals[:] = []
                    self.actual_score = 0
                    self.mem.set(self.mem.CURRENT_GOALS, None)
                    print("cost" + str(self.actual_cost))
                    print("score" + str(self.actual_score))

        # if there are current goals or pending goals then execute them
        # else folow selection
        if  self.mem.get(self.mem.CURRENT_GOALS) or self.selected_goals:
            goals = self.check_existing_goals()

        if not goals:
            # get all the goals from the root of the goal graph
            goals = goalGraph.getUnrestrictedGoals()
            # select the goals based on the score,time and cost
            if self.method == "selection_method":
                goals = self.selection_method(goals)
            else:
                goals = self.fifo_selection (goals)
            # add it to th current goal in memory
            self.mem.set(self.mem.CURRENT_GOALS, goals)

        # Time remaining
        if goals:
            print("Money Remaining: " + str(self.mem.get(self.mem.MONEY)))
        else:
            self.mem.set(self.mem.CURRENT_GOALS, [])
            print("Money Remaining: " + str(self.mem.get(self.mem.MONEY)))
            self.mem.set(self.mem.MONEY, 0)
            self.selected_goals[:] = []
            self.actual_score = 0
            self.actual_cost = 0

        if trace:
            trace.add_data("GOALS",goals)

        if not goals:
            if verbose >= 2:
                print "No goals selected."
        else:
            if verbose >= 2:
                print "Selecting goal(s):",
                for goal in goals:
                    print goal,
                print

class SimpleIntend_construction_goal_transformation(base.BaseModule):

    goals_goalgraph = []
    building_list = []
    current_goals_copy = []
    score = 0

    def remove_goals_from_goalgraph(self,goal):
        for goals in self.goals_goalgraph:
            if goal.args == goals.args and goal.kwargs["predicate"] == goals.kwargs["predicate"]:
                self.goals_goalgraph.remove(goals)

        return

    def check_arguments(self,arg , building):
        goals = copy.deepcopy(self.goals_goalgraph)
        for goal in goals:
            if arg in goal.args:
                building.append(goal)
                self.remove_goals_from_goalgraph(goal)
                self.check_arguments(goal.args[0] , building)
        return


    def building_blocks(self):
        building = []
        goals = copy.deepcopy(self.goals_goalgraph)
        for goal in goals:
            if(goal.kwargs["predicate"] == "on-table"):
                building[:] = []
                building.append(goal)
                self.remove_goals_from_goalgraph(goal)
                self.check_arguments(goal.args[0],building)
                self.building_list.append(copy.deepcopy(building))
        return

    def calculate_score(self):
        '''
        input: get the global variable current_goals_copy
        output : update the score ; for mortar it is 2 , else 1
        '''
        executed_buildings = []
        if self.mem.get(self.mem.ACTUAL_SCORES):
            self.score = self.mem.get(self.mem.ACTUAL_SCORES)
        else:
            self.score = 0
        for goal in self.current_goals_copy :
            if goal["predicate" ] == "on":
                self.score = self.score + 1
            if goal["predicate"] == "stable-on":
                self.score = self.score + 2
            if goal["predicate"] == "on-table":
                temp = [goal.args[0] , len(self.current_goals_copy)]
                executed_buildings.append(tuple(temp))
        self.current_goals_copy[:] = []
        self.mem.set(self.mem.ACTUAL_SCORES , self.score)
        if self.mem.get(self.mem.EXECUTED_BUILDING_LIST):
            buildings = copy.deepcopy(self.mem.get(self.mem.EXECUTED_BUILDING_LIST))
            buildings.append(executed_buildings)
        else:
            buildings = copy.deepcopy(executed_buildings)
        self.mem.set(self.mem.EXECUTED_BUILDING_LIST, buildings)

    def selected_goals_to_memory(self):
        '''
        input : selected goals to memory variable select_building_list
        '''
        selected_buildings = []
        for each in self.building_list:
            for goal in each:
                if goal["predicate"] == "on-table":
                    temp = [goal.args[0] , len(each)]
                    selected_buildings.append(tuple(temp))
                    break
        self.mem.set(self.mem.SELECTED_BUILDING_LIST , selected_buildings)

    def run(self, cycle, verbose = 2):
        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("GOALGRAPH",copy.deepcopy(self.mem.GOAL_GRAPH))

        goalGraph = self.mem.get(self.mem.GOAL_GRAPH)

        goals = goalGraph.getUnrestrictedGoals()



        if not goalGraph or not goals:
            if verbose >= 1:
                print "Goal graph not initialized. Intend will do nothing."
                if not self.mem.get(self.mem.TIME_CONSTRUCTION):
                    self.goals_goalgraph[:] = []
                    self.building_list[:] = []
                    if self.current_goals_copy:
                        self.current_goals_copy[:] = []
                    else:
                        self.current_goals_copy = []
                    self.score = 0
                else:
                    if self.current_goals_copy:
                        self.calculate_score()
                    self.goals_goalgraph[:] = []
                    self.building_list[:] = []
                    self.current_goals_copy[:] = []
                    self.score = 0

            return

        if self.mem.get(self.mem.CURRENT_GOALS):
            self.current_goals_copy = copy.deepcopy(self.mem.get(self.mem.CURRENT_GOALS))
            goals = self.mem.get(self.mem.CURRENT_GOALS)
        else:

            if self.current_goals_copy:
                self.calculate_score()

            if not self.building_list:
                # get all the goals from the root of the goal graph
                self.score = 0
                goals = goalGraph.getUnrestrictedGoals()
                self.goals_goalgraph = copy.deepcopy(goals)
                self.building_blocks()
                if goals:
                    self.selected_goals_to_memory()
            # take the first building
            if self.building_list:
                goals = self.building_list.pop(0)

            self.current_goals_copy = copy.deepcopy(self.mem.get(self.mem.CURRENT_GOALS))

            # add it to the current goal in memory
            self.mem.set(self.mem.CURRENT_GOALS, goals)

        if trace:
            trace.add_data("GOALS",goals)

        if not goals:
            if verbose >= 2:
                print "No goals selected."
        else:
            if verbose >= 2:
                print "Selecting goal(s):",
                for goal in goals:
                    print goal,
                print

class SimpleIntend_construction(base.BaseModule):

    goals_goalgraph = []
    goals_in_goalgraph = []
    building_list = []
    selected_goals = []
    t = [1 , 2.2 , 3.4 , 5.4 , 8.4 , 13.4 , 22.4 , 42.4]
    buildings_scores = {'b' : [] , 't' : [] }
    combinations_t = { }
    combinations_b = { }
    selected_buildings = []
    time = None # for changing time very 20s
    time_change = False
    count = -1
    T = 0
    new_time = None
    goals = []
    actual_scores = 0
    new = False
    cycle = 0
    p_list = []
    t_list = []

    #def build_building_from_goals(base,goals):


    def remove_goals_from_goalgraph(self,goal):
        for goals in self.goals_goalgraph:
            if goal.args == goals.args and goal.kwargs["predicate"] == goals.kwargs["predicate"]:
                self.goals_goalgraph.remove(goals)

        return

    def check_arguments(self,arg , building):
        goals = copy.deepcopy(self.goals_goalgraph)
        for goal in goals:
            if arg in goal.args:
                building.append(goal)
                self.remove_goals_from_goalgraph(goal)
                self.check_arguments(goal.args[0] , building)
        return


    def building_blocks(self):
        building = []
        goals = copy.deepcopy(self.goals_goalgraph)
        for goal in goals:
            if(goal.kwargs["predicate"] == "on-table"):
                building[:] = []
                building.append(goal)
                self.remove_goals_from_goalgraph(goal)
                self.check_arguments(goal.args[0],building)
                self.building_list.append(copy.deepcopy(building))


        return


    def check_current_goals_cant_be_acheived(self):
        time = 0
        world = self.mem.get(self.mem.STATES)[-1]
        current_goals = self.mem.get(self.mem.CURRENT_GOALS)
        for i,goal in enumerate(current_goals):
            achieved = world.atom_true(world.midcaGoalAsAtom(goal))
            if not achieved:
                time = self.t[i]
        if time > self.T:
            return True
        else:
            return False

    def run(self, cycle, verbose = 2):
        self.goals_goalgraph = [] # store all goals in the goalgraph
        self.goals_in_goalgraph = [] # copy of self.goals_goalgraph
        self.building_list = [] # goals categorized as building goals
        # time
        self.t = [1 , 2.2 , 3.4 , 5.4 , 8.4 , 13.4 , 22.4 , 42.4]
        self.t_copy = [1 , 1.2 , 1.2 , 2 , 3 , 5 , 9 , 20]
        # b and time for the buildings
        self.buildings_scores = {'b' : [] , 't' : [] }
        self.combinations_t = { }
        self.combinations_b = { }
        self.selected_goals = []
        #self.selected_buildings = []
        self.T = self.mem.get(self.mem.TIME_CONSTRUCTION)


        time_variation = 0.2
        time_change = 0.5
        self.implement ="selection"
        #self.implement ="stack"



        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("GOALGRAPH",copy.deepcopy(self.mem.GOAL_GRAPH))

        goalGraph = self.mem.get(self.mem.GOAL_GRAPH)

        if len(self.mem.get(self.mem.GOAL_GRAPH).getAllGoals()) == 0:
            if verbose >= 1:
                print "No Goals in Goal graph. Intend will do nothing."
                return

        if not goalGraph:
            if verbose >= 1:
                print "Goal graph not initialized. Intend will do nothing."
            return




        # to check whether the goals are acheived or not.
        world = self.mem.get(self.mem.STATES)[-1]

        # calculate time
        if not self.time:
            self.time = ctime.time()

        # calculate cycles, for every cycle greater than 10
        # randomly change the construction time
        '''
        self.cycle = self.cycle + 1
        if self.cycle >= 10:
                self.cycle = 0
                self.T = random.uniform(time_change*(self.T), self.T + time_change*(self.T))
                print("Time is changed : " + str(self.T))
                self.mem.set(self.mem.TIME_CONSTRUCTION, self.T)
        '''

        if self.mem.get(self.mem.CURRENT_GOALS) and self.T :
            if not len(self.mem.get(self.mem.CURRENT_GOALS)) == 0:
            # check the acheived goals in current goals
            # if all goals acheived, temp = 0
            # add it to the executed_building_list in memory
            # i indicates how many goals acheived
            # self.count and i are same, except that when they are not equal a goal is acheived
                temp = 0
                i = 0
                for i,goal in enumerate(self.mem.get(self.mem.CURRENT_GOALS)):
                    achieved = world.atom_true(world.midcaGoalAsAtom(goal))
                    if not achieved:
                        temp = 1
                        break

                if temp == 1:
                    i= i-1

                if temp == 0:
                    self.count = -100
                    temp_build = self.mem.get(self.mem.EXECUTED_BUILDING_LIST)
                    if temp_build:
                        temp_build.append(len(self.mem.get(self.mem.CURRENT_GOALS)))
                        self.mem.set(self.mem.EXECUTED_BUILDING_LIST , temp_build )
                        print(self.mem.get(self.mem.EXECUTED_BUILDING_LIST))
                    else:
                        self.mem.set(self.mem.EXECUTED_BUILDING_LIST , [len(self.mem.get(self.mem.CURRENT_GOALS))])
                        print(self.mem.get(self.mem.EXECUTED_BUILDING_LIST))



                # if zone is -1, then re- initialize self.count to 0
                # if there is no sufficient time, then give a hint to eval to remove all goals
                zone = 0
                if not (self.count == i):
                    random.seed(i)
                    action_score = random.uniform(1 - time_variation*1,1 + time_variation*1)
                    action_time = random.uniform(self.t_copy[i] - time_variation*self.t_copy[i], self.t_copy[i] + time_variation*self.t_copy[i])
                    print("Actual Time Taken For Previous Action :" + str(action_time))
                    print("Expected Time Taken For Previous Action :" + str(self.t_copy[i]))
                    print("Actual Score for Previous Action : " + str(action_score))

                    #raw_input("enter")
                    self.actual_scores = self.actual_scores + action_score
                    actual_time  = self.mem.get(self.mem.ACTUAL_TIME_CONSTRUCTION)
                    temp_p = round(action_score,3)
                    temp_t = round(action_time,3)
                    self.p_list.append(temp_p)
                    self.t_list.append(temp_t)
                    if actual_time:
                        actual_time = actual_time + action_time
                        self.mem.set(self.mem.ACTUAL_TIME_CONSTRUCTION ,actual_time )
                    else:
                        self.mem.set(self.mem.ACTUAL_TIME_CONSTRUCTION ,action_time )
                    self.T = self.T - action_time
                    if self.count == -100:
                        #print("Zone 1")
                        zone = -1

                    if self.check_current_goals_cant_be_acheived():
                        if self.implement:
                            if self.implement is "selection":
                                self.p_list = []
                                self.t_list = []
                                self.mem.set(self.mem.CURRENT_GOALS , [])
                                #raw_input(self.count)
                                self.count = -1
                                self.actual_scores = 0
                                del self.selected_buildings


                    if self.T < 0:
                        #print("Zone 2")

                        if temp == 0:
                            temp_build = self.mem.get(self.mem.EXECUTED_BUILDING_LIST)
                            if temp_build:
                                temp_build.pop()
                                self.mem.set(self.mem.EXECUTED_BUILDING_LIST , temp_build)

                        self.T = 0
                        self.actual_scores = 0
                        if actual_time:
                            self.mem.set(self.mem.ACTUAL_TIME_CONSTRUCTION , actual_time - action_time)
                        print("No Goals are selected due to less time .Abandoning Goals in next cycle.")
                        self.mem.set(self.mem.CURRENT_GOALS , [])
                        self.mem.set(self.mem.REJECTED_GOALS,["remove all"])
                        self.p_list = []
                        self.t_list = []
                        self.goals =[]
                        if self.selected_buildings:
                            del self.selected_buildings
                        self.mem.set(self.mem.TIME_CONSTRUCTION , self.T)
                        self.count = -1
                        if zone == -1:
                            self.count = -1
                        return
                    P = self.mem.get(self.mem.P)
                    t = self.mem.get(self.mem.t)


                    if P :
                        if self.p_list:
                            if not self.p_list in P:
                                P.append(self.p_list)
                                self.mem.set(self.mem.P,P)
                    else:
                        if self.p_list:
                            self.mem.set(self.mem.P,self.p_list)

                    if t :
                        if self.t_list:
                            if not self.t_list in t:
                                t.append(self.t_list)
                                self.mem.set(self.mem.t,t)
                    else:
                        if self.t_list:
                            self.mem.set(self.mem.t,self.t_list)


                    self.mem.set(self.mem.TIME_CONSTRUCTION , self.T)
                    self.count = i
                    print("scores and time")
                    if zone == -1:
                        if self.mem.get(self.mem.ACTUAL_SCORES):
                            score = self.mem.get(self.mem.ACTUAL_SCORES) + self.actual_scores
                            self.mem.set(self.mem.ACTUAL_SCORES , score)
                        else:
                            self.mem.set(self.mem.ACTUAL_SCORES , self.actual_scores)
                        self.actual_scores = 0
                        self.p_list = []
                        self.t_list = []
                        self.count = -1




        print("Time Remaining : " + str(self.T))



        if ( (ctime.time() - self.time) > 120) and (self.T >= 0):
            self.time = ctime.time()
            self.T = random.uniform(0.7*(self.T), self.T)
            #self.time_change = True
            print("")
            print("Time is changed & Time Remaining is: " + str(self.T))
            self.mem.set(self.mem.TIME_CONSTRUCTION, self.T)
            print("")



        # get all the goals from the goal graph
        self.goals_goalgraph = goalGraph.getUnrestrictedGoals()

        # Just a copy of all the goals from goal graph
        self.goals_in_goalgraph = copy.deepcopy(self.goals_goalgraph )


        # if there are current goals to acheive, let it complete it's execution
        if  self.mem.get(self.mem.CURRENT_GOALS) :
            if not len(self.mem.get(self.mem.CURRENT_GOALS)) == 0:
                print("Not all Current goals achieved ,")
                print("Processing" + str(self.mem.get(self.mem.CURRENT_GOALS)))
                return

        # This function gets the goals as the building goals, and writes to the complete_building_list
        self.building_blocks()
        temp_building_list = []
        if not self.mem.get(self.mem.COMPLETE_BUILDING_LIST):
            for each in self.building_list:
                temp_building_list.append(len(each))
            self.mem.set(self.mem.COMPLETE_BUILDING_LIST , temp_building_list)


        print ("Given Building Goals")
        for i in range(0,len(self.building_list)):
            print("")
            print("Tower " + self.building_list[i][0].args[0])
            print ("[")
            for j in self.building_list[i]:
                print(j)
            print("]")
            print("")
        print("")


        p_t = [] # This is list variable with (P/T) scores of all buildings
        towers = []
        temp = []
        # this variable contains the buildings that are selected
        self.selected_buildings = self.building_list


        for i in range(0,len(self.building_list)):
            self.buildings_scores['t'].append(self.t[len(self.building_list[i])-1])
            self.buildings_scores['b'].append(len(self.building_list[i])/self.t[len(self.building_list[i])-1])
            p_t.append(len(self.building_list[i])/self.t[len(self.building_list[i])-1])

        # when there is no time limit
        # self.T is the time that you specify on Midca/examples/cogsci_demo_mortar_construction.py


        if self.implement is "stack":
            temp = []
            towers = []
            score = 0
            time = 0
            for i in range(0,len(self.building_list)):
                score = score + len(self.building_list[i])
                time = time + self.t[len(self.building_list[i])-1]
                if time < self.T:
                    temp.append(self.building_list[i])
                    towers.append(self.selected_buildings[i][0].args[0])
                else:
                    break



            # Finally the selected buildings are the one's stored in temp
            if not temp:
                print("No Goals are selected due to less time . Removing all goals from Goal Graph During Next Evaluate Phase.")
                self.mem.set(self.mem.REJECTED_GOALS,["remove all"])
                self.mem.set(self.mem.CURRENT_GOALS , [])
                if self.goals:
                    del self.goals
                    del self.selected_buildings
                self.count = -1
                return
            else:
                score = 0
                time = 0
                self.selected_buildings = temp
                self.selected_goals = self.selected_buildings[0]
                for i in range(0,len(self.selected_buildings)):
                    score = score + len(self.selected_buildings[i])
                    time = time + self.t[len(self.selected_buildings[i])-1]
                print("Selected Towers are : " + str(towers))
                print("")
                length = [len(each) for each in self.selected_buildings]
                if not self.mem.get(self.mem.SELECTED_BUILDING_LIST):
                    self.mem.set(self.mem.SELECTED_BUILDING_LIST , length)
                print("Overall Score : " + str(score))
                print("Overall Time is : " + str(time))
                if not self.mem.get(self.mem.EXPECTED_TIME_CONSTRUCTION):
                    self.mem.set(self.mem.EXPECTED_TIME_CONSTRUCTION , time)
                print("")

        elif not self.T:
            minimum_score = min(self.buildings_scores['b'])
            #print("P/T is " + str(minimum_score))
            index = self.buildings_scores['b'].index(minimum_score)
            #print("time is " + str(self.buildings_scores['t'][index]))
            #self.selected_goals = self.building_list[index]
            #self.selected_buildings.append(self.building_list[index])
            #print("Score is " + str(len(self.building_list[index])))


            p_t.sort()
            temp = []
            towers = []
            # sort the buildings according to the highest P/T scores and store it in temp
            # Get the first argument of the first building goal which will be the name of the tower
            # store all the names of the towers in towers[] variable in the order of their P/T scores

            for each in p_t:
                for i in range(0,len(self.selected_buildings)):
                    if each == (len(self.selected_buildings[i])/self.t[len(self.selected_buildings[i])-1]):
                        temp.append(self.selected_buildings[i])
                        towers.append(self.selected_buildings[i][0].args[0])

            # Finally the selected buildings are the one's stored in temp
            self.selected_buildings = temp
            self.selected_goals = self.selected_buildings[0]
            print("Selected Towers are : " + str(towers))
            print("")

        # when there is  time limit
        else:
            self.selected_buildings = []
            a = []
            for i in range(0,len(self.buildings_scores['t'])):
                a.append(i)
            for i in xrange(1,len(a)+1):
                c = list(itertools.combinations(a,i))
                for every in c:
                    sum=0
                    name = ""
                    for each in every:
                        sum = sum + self.buildings_scores['t'][each]
                        name = name + str(each) + ","
                    name = name[:-1]
                    if not (sum > self.T):
                        self.combinations_t[name] = sum
            sorted_x = sorted(self.combinations_t.items(), key=operator.itemgetter(1))

            max_length = 0
            for every in sorted_x:
                name = every[0].split(",")
                if(max_length < len(name)):
                    max_length = len(name)

            for every in sorted_x:
                name = every[0].split(",")
                sum=0
                if(len(name) == max_length):
                    for each in name:
                        sum = sum + self.buildings_scores['b'][int(each)]

                    self.combinations_b[every[0]] = sum


            sorted_x = sorted(self.combinations_b.items(), key=operator.itemgetter(1))

            if(len(sorted_x) == 0):
                print("No Goals are selected due to less time . Removing all goals from Goal Graph During Next Evaluate Phase.")
                print(" ")
                self.count = -1
                temp_goals = []
                for goals in self.goals_in_goalgraph:
                    temp_goals.append(goals)
                self.mem.set(self.mem.REJECTED_GOALS,["remove all"])
                self.mem.set(self.mem.CURRENT_GOALS , [])
                if self.goals:
                    del self.goals
                    del self.selected_buildings
                return



            #print(sorted_x)
            score = 0;
            p_t = []
            towers = []
            time = 0
            for every in sorted_x[0]:
                name = every.split(",")
                for each in name:
                    self.selected_buildings.append(self.building_list[int(each)])
                    p_t.append(len(self.building_list[int(each)])/self.t[len(self.building_list[int(each)])-1])
                    score = score + len(self.building_list[int(each)])
                    #for goal in (self.building_list[int(each)]):
                    #       self.selected_goals.append(goal)
                #print("P/T is " + str(self.combinations_b[every]))
                time =  time + self.combinations_t[every]
                #print("Score is " + str(self.combinations_b[every] * self.combinations_t[every]))

                break

            p_t.sort()
            p_t.reverse()
            temp = []
            towers = []
            for each in p_t:
                for i in range(0,len(self.selected_buildings)):
                    if each == (len(self.selected_buildings[i])/self.t[len(self.selected_buildings[i])-1]):
                        if not self.selected_buildings[i][0].args[0] in towers:
                            temp.append(self.selected_buildings[i])
                            towers.append(self.selected_buildings[i][0].args[0])


            length = [len(each) for each in self.selected_buildings]
            if not self.mem.get(self.mem.SELECTED_BUILDING_LIST):
                self.mem.set(self.mem.SELECTED_BUILDING_LIST , length)
            self.selected_buildings = temp
            print("Selected Towers are : " + str(towers))
            print("Overall Score : " + str(score))
            print("Overall Time is : " + str(time))
            if not self.mem.get(self.mem.EXPECTED_TIME_CONSTRUCTION):
                self.mem.set(self.mem.EXPECTED_TIME_CONSTRUCTION , time)
            print("")




            selected_goals_mem = []
            if self.selected_buildings:
                #print("Selected Buildings")
                for each in self.selected_buildings:
                    for goal in each:
                        selected_goals_mem.append(goal)
                    #print("")
                self.selected_goals = self.selected_buildings[0]
                #self.mem.set(self.mem.SELECTED_GOALS, selected_goals_mem)

        '''

        for i in range(0,len(self.building_list)):
                for j in self.building_list[i]:
                        print(j)
                print("")
        '''

        self.mem.set(self.mem.CURRENT_GOALS, copy.deepcopy(self.selected_goals))
        #print(self.mem.get(self.mem.CURRENT_GOALS))

        if trace:
            trace.add_data("GOALS",copy.deepcopy(self.selected_goals))
            trace.add_data("GOALGRAPH",copy.deepcopy(self.mem.GOAL_GRAPH))

        if not self.selected_goals:
            if verbose >= 2:
                print "No goals selected."
        else:
            if verbose >= 2:
                print("Time is " + str(self.T))
                print("")
                print "Selecting Tower(s):",
                #print("")


                for i in range(0,len(self.selected_buildings)):
                    print("")
                    print("Tower " + self.selected_buildings[i][0].args[0] ),
                    if(len(self.selected_buildings[i]) == 1):
                        print("(" + str(len(self.selected_buildings[i])) + " Block)"),
                    else:
                        print("(" + str(len(self.selected_buildings[i])) + " Blocks)"),
                    print(" with priority : " + str(i+1)),
                    print ("["),
                    #for goals in self.selected_buildings[i]:
                    #       print(goals)
                    #print("]")
                    #print("")
                    print("Time(T) :" + str(self.t[len(self.selected_buildings[i])-1]) + ","),
                    print("Score(P) :" + str(len(self.selected_buildings[i])) + ","),
                    print("P/T :" + str(len(self.selected_buildings[i])/self.t[len(self.selected_buildings[i])-1]) + " ]"),
                print("")


                self.goals = copy.deepcopy(self.selected_buildings[0])
                del self.selected_buildings[0]

                '''
                for goal in self.selected_goals:
                    print goal,
                print
                '''
                print("")

class BestHillClimbingIntendGraceNSF(base.BaseModule):

    def __init__(self, experiment="None"):
        # for hill climbing
        self.previous_goal = []
        self.experiment = experiment

    def displayEachGoalTrajectory(self, GoalTrajectory):
        print ("*******************STATES*******************")
        for atom in GoalTrajectory[0]:
            print atom
        print ("*******************GOALS*******************")
        for goal in GoalTrajectory[1]:
            print goal
        print("**********************************************")

    def displayGoalTrajectory(self):
        """
        print goal agend a history
        """
        print ("*******************Goal Trajectory*******************")
        GoalTrajectory = self.mem.get(self.mem.GoalTrajectory)
        for eachGoalTrajectory in GoalTrajectory:
            self.displayEachGoalTrajectory(eachGoalTrajectory)
        print ("*******************End Goal Trajectory*******************")

    def parse_tile(self, input):
        output = ""
        y_index = input.index('y')
        output = [int(input[2:y_index]), int(input[y_index + 1:])]
        return output

    def get_adjacent_goals(self, goal, all_goals):
        """
        :param goal: a goal of format (surveyed, subarea-5, Tx0y0)
        :param goals: list of goals from goalgraph
        :return: return a list of adjacent goals
                         ex:
                           (surveyed, subarea-5, Tx0y1)
                           (surveyed, subarea-5, Tx1y0)
        """
        adjacent_goals = []
        copy_goal = copy.deepcopy(goal)

        # get the x and y coordinate from the goal
        [x, y] = self.parse_tile(copy_goal.args[1])

        arguments = list(copy_goal.args)
        arguments[1] = "Tx" + str(x + 1) + "y" + str(y)
        copy_goal.args = tuple(arguments)

        if copy_goal in all_goals:
            # to get the object first we need to get the index
            index = all_goals.index(copy_goal)
            adjacent_goals.append(all_goals[index])

        arguments = list(copy_goal.args)
        arguments[1] = "Tx" + str(x) + "y" + str(y+1)
        copy_goal.args = tuple(arguments)

        if copy_goal in all_goals:
            # to get the object first we need to get the index
            index = all_goals.index(copy_goal)
            adjacent_goals.append(all_goals[index])

        arguments = list(copy_goal.args)
        arguments[1] = "Tx" + str(x - 1) + "y" + str(y)
        copy_goal.args = tuple(arguments)

        if copy_goal in all_goals:
            # to get the object first we need to get the index
            index = all_goals.index(copy_goal)
            adjacent_goals.append(all_goals[index])

        arguments = list(copy_goal.args)
        arguments[1] = "Tx" + str(x) + "y" + str(y-1)
        copy_goal.args = tuple(arguments)

        if copy_goal in all_goals:
            # to get the object first we need to get the index
            index = all_goals.index(copy_goal)
            adjacent_goals.append(all_goals[index])

        return adjacent_goals

    def createGraph(self, goals):
        """
        :param goals: list of goals from goalgraph
        :return: return a connected graph
        """
        graph = {}

        # get the previous and the adjacent are in graph
        for goal in goals:
            graph[goal] = self.get_adjacent_goals(goal, goals)

        return graph

    def get_score(self, goal, world):
        """
        :param goal: the goal in predicate(args*)
        :param world: states of the world
        :return: fish tag estimates of the position
        """
        for atom in world.atoms:
            if atom.predicate.name == "estuniqueTagCount":
                if goal.args[1] == atom.args[2].name:
                    return float(atom.args[1].name)

        #return 0

    def createScoreGraph(self, goals, world):
        """
        :param goals: list of goals from goalgraph
               world: obj of worldsim.world
        :return: return a graph with key as goals and score as estimates
        """
        graph = {}

        # get the previous and the adjacent are in graph
        for goal in goals:
            graph[goal] = self.get_score(goal, world)

        return graph

    def get_actual_score(self, goal, world):
        """
        :param goal: the goal in predicate(args*)
        :param world: states of the world
        :return: fish tag estimates of the position
        """
        for atom in world.atoms:
            if atom.predicate.name == "uniqueTagCount":
                if goal.args[1] == atom.args[2].name:
                    return float(atom.args[1].name)

        #return 0

    def createActualScoreGraph(self, goals, world):
        """
        :param goals: list of goals from goalgraph
               world: obj of worldsim.world
        :return: return a graph with key as goals and score as estimates
        """
        graph = {}

        # get the previous and the adjacent are in graph
        for goal in goals:
            graph[goal] = self.get_actual_score(goal, world)

        return graph

    # Function to print a BFS of graph
    def bfs(self, graph, s, visited = []):

        order = []
        visited = {}
        # Mark all the vertices as not visited
        for each in graph:
            visited[each] = False

        #visited = [False] * (len(graph))

        # Create a queue for BFS
        queue = []

        # Mark the source node as
        # visited and enqueue it
        queue.append(s)
        order.append(s)
        visited[s] = True

        while queue:

            # Dequeue a vertex from
            # queue and print it
            s = queue.pop(0)
            order.append(s)
            print (s)

            # Get all adjacent vertices of the
            # dequeued vertex s. If a adjacent
            # has not been visited, then mark it
            # visited and enqueue it
            for i in graph[s]:
                if visited[i] == False:
                    queue.append(i)
                    visited[i] = True

        return order

    def dfs(self, graph, neighbour, visited=[]):
        """
        :param graph: connected graph of goals
        :param goal: start goal
        :return: return a list of goals
        """
        if neighbour not in visited:
            #print (neighbour)
            visited.append(neighbour)
            for neighbour in graph[neighbour]:
                self.dfs(graph, neighbour, visited)

        return visited

    def Searchindfs(self, goals, world):
        """
        :param goals: goals of the grace
        :param world: object of the worldsim.world
        :return: list of goals in dfs
        """

        # get the current state of the agent
        [atom] = world.get_atoms(["agent-at", "grace"])
        start_goal = None

        # get the start goal from where the agent is in
        for goal in goals:
            if goal.args[1] == atom.args[1].name:
                start_goal = goal
                break
        if start_goal:
            graph = self.createGraph(goals)
            return self.dfs(graph, start_goal, visited=[])
        else:
            # create a dummy goal where the agent is
            # so that you will have better connected graph
            # since that goal is dummy add it in visited
            start_goal = dummygoalgen.Goal(*["grace", atom.args[1].name], predicate='surveyed')
            goals.append(start_goal)
            graph = self.createGraph(goals)
            visited = self.dfs(graph, start_goal, visited=[])
            visited.remove(start_goal)
            while start_goal in visited:
                visited.remove(start_goal)
            return visited

    def get_max_neighbor(self, graph, score_graph, goal):
        """

        :param graph: connected graph with goals
        :param score_graph: scores for the goals
        :param start_goal: goal to start with
        :param previous_goal: goal previously achieved
        :return: goal
        """

        # check the scores of the neighbours of the previous goal an start goal
        # select the one with highest score

        max_score = -1
        selected_goal = None
        previous_goals = []

        # get all the neighbors of start and previous goal
        all_neighbours = graph[goal]

        #convert previous_goals to string
        if self.previous_goal:
            previous_goals = [str(prevgoal) for prevgoal in self.previous_goal]

        # check which one has maximum score
        for goal in all_neighbours:
            if not str(goal) in previous_goals:
                if max_score <= score_graph[goal]:
                    max_score = score_graph[goal]
                    selected_goal = goal

        return selected_goal

    def lookalike(self, goal, graph):
        for each in graph:
            if str(each) == str(goal):
                return each

    def min_distance_goal(self, goals, atom):
        """
        :param goals: goals
        :param atom: agent-at(grace, "")
        :return: a goal which is minimum distance from the agent
        """
        agent_location = self.parse_tile(atom.args[1].name)
        agent_location = np.array((agent_location[0],agent_location[1]))
        min_dist_goal = None
        distance = float('inf')
        for goal in goals:
            dest_location = self.parse_tile(goal.args[1])
            goal_location = np.array((dest_location[0], dest_location[1]))
            dist_goal_agent = np.linalg.norm(agent_location - goal_location)
            if distance > dist_goal_agent:
                distance = dist_goal_agent
                min_dist_goal = goal
        return min_dist_goal


    def sortbyscores(self, previous_goals, score_graph):
        sorted_goals = []
        goals = sorted(score_graph, key=score_graph.get)
        for each in goals:
            if each in previous_goals:
                sorted_goals.append(each)

        return sorted_goals



    def hillclimbing(self, graph, score_graph, actual_score_graph,
                     previous_goals, start_goal):
        """

        :param graph: connected graph with goals
        :param score_graph: expected scores for the goals
        :param actual_score_graph: actual scores for the goals
        :param start_goal: recently completed goal
        :param previous_goals: goals previously achieved without start_goal
        :return: goal
        """
        # if there are no previous trajectory
        if not previous_goals:
            goal = self.get_max_neighbor(graph, score_graph, start_goal)
            return goal

        # if there is a trajectory
        # check the actual values of the previous goal and the previous previous one
        else:
            # previous previous one
            previous_goal = previous_goals[-1]

            # get a look alike goal from the actual_score_graph
            previous_goal = self.lookalike(previous_goal, actual_score_graph)
            start_goal = self.lookalike(start_goal, actual_score_graph)

            # if the previous previous one is greater than previous goal backtrack
            if actual_score_graph[previous_goal] >= actual_score_graph[start_goal]:

                # back tracking
                goal = self.get_max_neighbor(graph, score_graph, previous_goal)

                if goal:
                    # update previous goal
                    return goal
                # if there are no unexplored ones
                else:
                    previous_goals.pop()
                    return self.hillclimbing(graph, score_graph, actual_score_graph,
                                 previous_goals, start_goal)
            else:
                goal = self.get_max_neighbor(graph, score_graph, start_goal)
                if goal:
                    return goal
                else:
                    # back track to the one that has highest score
                    start_goal = previous_goals.pop()
                    return self.hillclimbing(graph, score_graph, actual_score_graph,
                                 previous_goals, start_goal)


    def Searchinhillclimbing(self, goals, world):
        """
        :param goals: goals of the grace
        :param world: object of the worldsim.world
        :return: list of goals in dfs
        """
        if not goals:
            return [None]
        """"   
        # get the current state of the agent
        [atom] = world.get_atoms(["agent-at", "grace"])
        start_goal = None

        # if there is only one goal then make it the start goal
        if len(goals) == 1:
            start_goal = goals[0]

        # get the start goal from where the agent is in
        for goal in goals:
            if goal.args[1] == atom.args[1].name:
                start_goal = goal
                break

        if start_goal:
            self.previous_goal.append(copy.deepcopy(start_goal))
            return [start_goal]

        elif not start_goal and not self.previous_goal:
            start_goal = goals[0]
            self.previous_goal.append(copy.deepcopy(start_goal))
            return [start_goal]
        
        if len(goals) == 1:
            start_goal = goals[0]
            self.previous_goal.append(copy.deepcopy(start_goal))
            return [start_goal]
        """
        if not self.previous_goal:
            [atom] = world.get_atoms(["agent-at", "grace"])
            start_goal = self.min_distance_goal(goals, atom)
            self.previous_goal.append(copy.deepcopy(start_goal))
            return [start_goal]
        else:
            original_goals = goals[:]

            # create a dummy goal where the agent is
            # so that you will have better connected graph
            # since that goal is dummy add it in visited
            previous_goals = copy.deepcopy(self.previous_goal)

            # for connected graph
            for each in previous_goals:
                    goals.append(each)

            # previously achieved goal
            start_goal = previous_goals.pop()

            # create graph
            graph = self.createGraph(goals)

            # if there are no neighbours then return the


            # create estimated score graph
            score_graph = self.createScoreGraph(goals, world)

            # create actual score graph
            actual_score_graph = self.createActualScoreGraph(goals, world)

            # sort previous goals by values in actual_score_graph
            previous_goals = self.sortbyscores(previous_goals, actual_score_graph)


            goal = self.hillclimbing(graph, score_graph, actual_score_graph, previous_goals, start_goal)

            if goal == None:
                [atom] = world.get_atoms(["agent-at", "grace"])
                goal = self.min_distance_goal(original_goals, atom)
                self.previous_goal.append(copy.deepcopy(goal))
            else:
                self.previous_goal.append(copy.deepcopy(goal))

            return [goal]

    def filter(self, goals, predicatename):
        filtered_goals = []
        for goal in goals:
            if self.experiment == "selection" or self.experiment == "smart":
                if goal['predicate'].startswith(predicatename)\
                        and not (goal['predicate'] == "surveyed-ergodic"):
                    filtered_goals.append(goal)
            else:
                if goal['predicate'] == predicatename\
                        and not (goal['predicate'] == "surveyed-ergodic"):
                    filtered_goals.append(goal)
        return filtered_goals

    def check_failed_goals(self, current_goals):

        if current_goals:
            current_goals_copy = copy.deepcopy(current_goals[-1])
            for goal in current_goals_copy:
                if goal.status == False:
                    current_goals = None

        return current_goals

    def run(self, cycle, verbose=2):
        trace = self.mem.trace
        world = self.mem.get(self.mem.STATES)[-1]
        if trace:
            trace.add_module(cycle, self.__class__.__name__)
            trace.add_data("GOALGRAPH", copy.deepcopy(self.mem.GOAL_GRAPH))

        goalGraph = self.mem.get(self.mem.GOAL_GRAPH)

        if not goalGraph:
            if verbose >= 1:
                print "Goal graph not initialized. Intend will do nothing."
            return
        # get all the goals from the goal graph
        goals = goalGraph.getUnrestrictedGoals()
        filtered_goals = self.filter(goals, "surveyed")
        suspended_goals = []
        if self.experiment == "selection" or self.experiment == "smart":
            goals  = [goal for goal in filtered_goals if goal.status == True]
            suspended_goals = [goal for goal in filtered_goals if goal.status == False]
            if not goals and suspended_goals:
                goals = suspended_goals
        if not goals:
            if verbose >= 1:
                print "No Survey goals. Intend will not select goals via hillclimbing."
            return

        current_goals = self.mem.get(self.mem.CURRENT_GOALS)

        if self.experiment == "selection" or self.experiment == "smart":
            current_goals = self.check_failed_goals(current_goals)

        need_for_hillclimbing_flag = True
        if current_goals:
            #get the top of the stack goals
            current_goals = current_goals[-1]
            if current_goals[0]["predicate"] == "inspected":
                need_for_hillclimbing_flag = False
            #check if it is a surveyed goal and if the agent is in the same cell
            exec_goals = self.filter(current_goals, "surveyed")
            if exec_goals:
                #check if any of it is in the same agent location
                [atom] = world.get_atoms(["agent-at", "grace"])
                for goal in exec_goals:
                    if goal.args[1] == atom.args[1].name:
                        need_for_hillclimbing_flag = False
                        break
            else:
                need_for_hillclimbing_flag = False

            if exec_goals and need_for_hillclimbing_flag:
                for goal in exec_goals:
                    if goal in self.previous_goal:
                        self.previous_goal.remove(goal)

        # current goals as a stack
        if not need_for_hillclimbing_flag:
            goals = current_goals
        else:
            goals = self.filter(goals, "surveyed")
            goals = self.Searchinhillclimbing(goals, world)
            if goals and not goals[0] == None:
                goals = [goals[0]]
                self.mem.set(self.mem.CURRENT_GOALS, [goals])
            else:
                if verbose >= 1:
                    print "Intend could not select a goal."

        if trace:
            trace.add_data("GOALS", goals)

        if not goals:
            if verbose >= 2:
                print "No goals selected."
        else:
            # add it to goaltrajectory for representation
            #world = self.mem.get(self.mem.STATES)[-1]
            # get_atoms = [atom for atom in world.atoms]
            # self.mem.add(self.mem.GoalTrajectory, [copy.deepcopy(get_atoms), copy.deepcopy(goals)])
            # self.displayGoalTrajectory()
            if verbose >= 2:
                print "Selecting goal(s):",
                for goal in goals:
                    print goal,
                print

class BestHillClimbingIntendGraceNSFOld(base.BaseModule):

    def __init__(self):
        # for hill climbing
        self.previous_goal = []

    def displayEachGoalTrajectory(self, GoalTrajectory):
        print ("*******************STATES*******************")
        for atom in GoalTrajectory[0]:
            print atom
        print ("*******************GOALS*******************")
        for goal in GoalTrajectory[1]:
            print goal
        print("**********************************************")

    def displayGoalTrajectory(self):
        """
        print goal agend a history
        """
        print ("*******************Goal Trajectory*******************")
        GoalTrajectory = self.mem.get(self.mem.GoalTrajectory)
        for eachGoalTrajectory in GoalTrajectory:
            self.displayEachGoalTrajectory(eachGoalTrajectory)
        print ("*******************End Goal Trajectory*******************")

    def parse_tile(self, input):
        output = ""
        y_index = input.index('y')
        output = [int(input[2:y_index]), int(input[y_index + 1:])]
        return output

    def get_adjacent_goals(self, goal, all_goals):
        """
        :param goal: a goal of format (surveyed, subarea-5, Tx0y0)
        :param goals: list of goals from goalgraph
        :return: return a list of adjacent goals
                         ex:
                           (surveyed, subarea-5, Tx0y1)
                           (surveyed, subarea-5, Tx1y0)
        """
        adjacent_goals = []
        copy_goal = copy.deepcopy(goal)

        # get the x and y coordinate from the goal
        [x, y] = self.parse_tile(copy_goal.args[1])

        arguments = list(copy_goal.args)
        arguments[1] = "Tx" + str(x + 1) + "y" + str(y)
        copy_goal.args = tuple(arguments)

        if copy_goal in all_goals:
            # to get the object first we need to get the index
            index = all_goals.index(copy_goal)
            adjacent_goals.append(all_goals[index])

        arguments = list(copy_goal.args)
        arguments[1] = "Tx" + str(x) + "y" + str(y+1)
        copy_goal.args = tuple(arguments)

        if copy_goal in all_goals:
            # to get the object first we need to get the index
            index = all_goals.index(copy_goal)
            adjacent_goals.append(all_goals[index])

        arguments = list(copy_goal.args)
        arguments[1] = "Tx" + str(x - 1) + "y" + str(y)
        copy_goal.args = tuple(arguments)

        if copy_goal in all_goals:
            # to get the object first we need to get the index
            index = all_goals.index(copy_goal)
            adjacent_goals.append(all_goals[index])

        arguments = list(copy_goal.args)
        arguments[1] = "Tx" + str(x) + "y" + str(y-1)
        copy_goal.args = tuple(arguments)

        if copy_goal in all_goals:
            # to get the object first we need to get the index
            index = all_goals.index(copy_goal)
            adjacent_goals.append(all_goals[index])

        return adjacent_goals

    def createGraph(self, goals):
        """
        :param goals: list of goals from goalgraph
        :return: return a connected graph
        """
        graph = {}

        # get the previous and the adjacent are in graph
        for goal in goals:
            graph[goal] = self.get_adjacent_goals(goal, goals)

        return graph

    def get_score(self, goal, world):
        """
        :param goal: the goal in predicate(args*)
        :param world: states of the world
        :return: fish tag estimates of the position
        """
        for atom in world.atoms:
            if atom.predicate.name == "estuniqueTagCount":
                if goal.args[1] == atom.args[2].name:
                    return float(atom.args[1].name)

        #return 0

    def createScoreGraph(self, goals, world):
        """
        :param goals: list of goals from goalgraph
               world: obj of worldsim.world
        :return: return a graph with key as goals and score as estimates
        """
        graph = {}

        # get the previous and the adjacent are in graph
        for goal in goals:
            graph[goal] = self.get_score(goal, world)

        return graph

    def get_actual_score(self, goal, world):
        """
        :param goal: the goal in predicate(args*)
        :param world: states of the world
        :return: fish tag estimates of the position
        """
        for atom in world.atoms:
            if atom.predicate.name == "uniqueTagCount":
                if goal.args[1] == atom.args[2].name:
                    return float(atom.args[1].name)

        #return 0

    def createActualScoreGraph(self, goals, world):
        """
        :param goals: list of goals from goalgraph
               world: obj of worldsim.world
        :return: return a graph with key as goals and score as estimates
        """
        graph = {}

        # get the previous and the adjacent are in graph
        for goal in goals:
            graph[goal] = self.get_actual_score(goal, world)

        return graph

    # Function to print a BFS of graph
    def bfs(self, graph, s, visited = []):

        order = []
        visited = {}
        # Mark all the vertices as not visited
        for each in graph:
            visited[each] = False

        #visited = [False] * (len(graph))

        # Create a queue for BFS
        queue = []

        # Mark the source node as
        # visited and enqueue it
        queue.append(s)
        order.append(s)
        visited[s] = True

        while queue:

            # Dequeue a vertex from
            # queue and print it
            s = queue.pop(0)
            order.append(s)
            print (s)

            # Get all adjacent vertices of the
            # dequeued vertex s. If a adjacent
            # has not been visited, then mark it
            # visited and enqueue it
            for i in graph[s]:
                if visited[i] == False:
                    queue.append(i)
                    visited[i] = True

        return order

    def dfs(self, graph, neighbour, visited=[]):
        """
        :param graph: connected graph of goals
        :param goal: start goal
        :return: return a list of goals
        """
        if neighbour not in visited:
            #print (neighbour)
            visited.append(neighbour)
            for neighbour in graph[neighbour]:
                self.dfs(graph, neighbour, visited)

        return visited

    def Searchindfs(self, goals, world):
        """
        :param goals: goals of the grace
        :param world: object of the worldsim.world
        :return: list of goals in dfs
        """

        # get the current state of the agent
        [atom] = world.get_atoms(["agent-at", "grace"])
        start_goal = None

        # get the start goal from where the agent is in
        for goal in goals:
            if goal.args[1] == atom.args[1].name:
                start_goal = goal
                break
        if start_goal:
            graph = self.createGraph(goals)
            return self.dfs(graph, start_goal, visited=[])
        else:
            # create a dummy goal where the agent is
            # so that you will have better connected graph
            # since that goal is dummy add it in visited
            start_goal = dummygoalgen.Goal(*["grace", atom.args[1].name], predicate='surveyed')
            goals.append(start_goal)
            graph = self.createGraph(goals)
            visited = self.dfs(graph, start_goal, visited=[])
            visited.remove(start_goal)
            while start_goal in visited:
                visited.remove(start_goal)
            return visited

    def get_max_neighbor(self, graph, score_graph, goal):
        """

        :param graph: connected graph with goals
        :param score_graph: scores for the goals
        :param start_goal: goal to start with
        :param previous_goal: goal previously achieved
        :return: goal
        """

        # check the scores of the neighbours of the previous goal an start goal
        # select the one with highest score

        max_score = -1
        selected_goal = None
        previous_goals = []

        # get all the neighbors of start and previous goal
        all_neighbours = graph[goal]

        #convert previous_goals to string
        if self.previous_goal:
            previous_goals = [str(prevgoal) for prevgoal in self.previous_goal]

        # check which one has maximum score
        for goal in all_neighbours:
            if not str(goal) in previous_goals:
                if max_score <= score_graph[goal]:
                    max_score = score_graph[goal]
                    selected_goal = goal

        return selected_goal

    def lookalike(self, goal, graph):
        for each in graph:
            if str(each) == str(goal):
                return each


    def sortbyscores(self, previous_goals, score_graph):
        sorted_goals = []
        goals = sorted(score_graph, key=score_graph.get)
        for each in goals:
            if each in previous_goals:
                sorted_goals.append(each)

        return sorted_goals



    def hillclimbing(self, graph, score_graph, actual_score_graph,
                     previous_goals, start_goal):
        """

        :param graph: connected graph with goals
        :param score_graph: expected scores for the goals
        :param actual_score_graph: actual scores for the goals
        :param start_goal: recently completed goal
        :param previous_goals: goals previously achieved without start_goal
        :return: goal
        """
        # if there are no previous trajectory
        if not previous_goals:
            goal = self.get_max_neighbor(graph, score_graph, start_goal)
            #update previous goal
            self.previous_goal.append(copy.deepcopy(goal))
            return goal

        # if there is a trajectory
        # check the actual values of the previous goal and the previous previous one
        else:
            # previous previous one
            previous_goal = previous_goals[-1]

            # get a look alike goal from the actual_score_graph
            previous_goal = self.lookalike(previous_goal, actual_score_graph)
            start_goal = self.lookalike(start_goal, actual_score_graph)

            # if the previous previous one is greater than previous goal backtrack
            if actual_score_graph[previous_goal] >= actual_score_graph[start_goal]:

                # back tracking
                goal = self.get_max_neighbor(graph, score_graph, previous_goal)

                if goal:
                    # update previous goal
                    return goal
                # if there are no unexplored ones
                else:
                    previous_goals.pop()
                    return self.hillclimbing(graph, score_graph, actual_score_graph,
                                 previous_goals, start_goal)
            else:
                goal = self.get_max_neighbor(graph, score_graph, start_goal)
                if goal:
                    return goal
                else:
                    # back track to the one that has highest score
                    start_goal = previous_goals.pop()
                    return self.hillclimbing(graph, score_graph, actual_score_graph,
                                 previous_goals, start_goal)


    def Searchinhillclimbing(self, goals, world):
        """
        :param goals: goals of the grace
        :param world: object of the worldsim.world
        :return: list of goals in dfs
        """

        # get the current state of the agent
        [atom] = world.get_atoms(["agent-at", "grace"])
        start_goal = None
        # get the start goal from where the agent is in
        for goal in goals:

            if goal.args[1] == atom.args[1].name:
                start_goal = goal
                break
        if start_goal:
            self.previous_goal.append(copy.deepcopy(start_goal))
            return [start_goal]
        else:
            # create a dummy goal where the agent is
            # so that you will have better connected graph
            # since that goal is dummy add it in visited
            previous_goals = copy.deepcopy(self.previous_goal)

            # for connected graph
            for each in previous_goals:
                goals.append(each)

            # previously achieved goal
            start_goal = previous_goals.pop()

            # create graph
            graph = self.createGraph(goals)

            # create estimated score graph
            score_graph = self.createScoreGraph(goals, world)

            # create actual score graph
            actual_score_graph = self.createActualScoreGraph(goals, world)

            # sort previous goals by values in actual_score_graph
            previous_goals = self.sortbyscores(previous_goals, actual_score_graph)


            goal = self.hillclimbing(graph, score_graph, actual_score_graph, previous_goals, start_goal)

            self.previous_goal.append(copy.deepcopy(goal))

            return [goal]

    def filter(self, goals, predicatename):
        filtered_goals = []
        for goal in goals:
            if goal['predicate'] == predicatename:
                filtered_goals.append(goal)
        return filtered_goals

    def run(self, cycle, verbose=2):
        trace = self.mem.trace
        world = self.mem.get(self.mem.STATES)[-1]
        if trace:
            trace.add_module(cycle, self.__class__.__name__)
            trace.add_data("GOALGRAPH", copy.deepcopy(self.mem.GOAL_GRAPH))

        goalGraph = self.mem.get(self.mem.GOAL_GRAPH)

        if not goalGraph:
            if verbose >= 1:
                print "Goal graph not initialized. Intend will do nothing."
            return
        # get all the goals from the goal graph
        goals = goalGraph.getAllGoals()
        if not goals:
            if verbose >= 1:
                print "No Goals in Goal graph. Intend will do nothing."
            return

        # current goals as a stack
        if self.mem.get(self.mem.CURRENT_GOALS):
            goals = None
        else:
            goals = self.filter(goals, "surveyed")
            goals = self.Searchinhillclimbing(goals, world)
            goals = [goals[0]]
            self.mem.set(self.mem.CURRENT_GOALS, [goals])

        if trace:
            trace.add_data("GOALS", goals)

        if not goals:
            if verbose >= 2:
                print "No goals selected."
        else:
            pass
            # add it to goaltrajectory for representation
            #world = self.mem.get(self.mem.STATES)[-1]
            # get_atoms = [atom for atom in world.atoms]
            # self.mem.add(self.mem.GoalTrajectory, [copy.deepcopy(get_atoms), copy.deepcopy(goals)])
            # self.displayGoalTrajectory()
            if verbose >= 2:
                print "Selecting goal(s):",
                for goal in goals:
                    print goal,
                print

class PriorityIntend(base.BaseModule):

    def run(self, cycle, verbose=2):
        trace = self.mem.trace
        if trace:
            trace.add_module(cycle, self.__class__.__name__)
            trace.add_data("GOALGRAPH", copy.deepcopy(self.mem.GOAL_GRAPH))

        goalGraph = self.mem.get(self.mem.GOAL_GRAPH)

        if not goalGraph:
            if verbose >= 1:
                print "Goal graph not initialized. Intend will do nothing."
            return
        # get all the goals from the root of the goal graph
        goals = goalGraph.getUnrestrictedGoals()

        if not goals:
            if verbose >= 1:
                print "No Goals in Goal graph. Intend will do nothing."
            return

        current_goals = self.mem.get(self.mem.CURRENT_GOALS)

        if not current_goals:
            # take the first goal
            pass
            # add it to the current goal in memory
        elif not current_goals[-1]:
            pass
        else:
            current_goals = current_goals[-1]
            # prioritize the goals with highest priority
            priority_goals = []
            for goal in goals:
                if goalGraph.cmp(goal, current_goals[0]) < 0:
                    priority_goals.append(goal)

            if priority_goals:
                goals = [priority_goals[-1]]
            else:
                goals = current_goals


        # current goals as a stack
        if self.mem.get(self.mem.CURRENT_GOALS):
            current_goals = self.mem.get(self.mem.CURRENT_GOALS)
            if not current_goals[-1] == goals:
                current_goals.append(goals)
                self.mem.set(self.mem.CURRENT_GOALS, current_goals)
        else:
            self.mem.set(self.mem.CURRENT_GOALS, [goals])

        if trace:
            trace.add_data("GOALS", goals)

        if not goals:
            if verbose >= 2:
                print "No goals selected."
        else:
            if verbose >= 2:
                print "Selecting goal(s):",
                for goal in goals:
                    print goal,
                print

class MetaIntend(base.BaseModule):

    def __init__(self):
        self.previous_anomolous_goals = []
        self.previous_goal = None

    def set_anomolous_goals(self, current_goal):

        if len(self.previous_anomolous_goals) >= 3:
            self.previous_anomolous_goals[0].status = True
            self.previous_anomolous_goals.pop(0)

        if not current_goal in self.previous_anomolous_goals:
            self.previous_anomolous_goals.append(current_goal)

    def remove_anomolous_goals(self):
       removed_goals = []
       for goal in self.previous_anomolous_goals:
           if not goal in self.mem.get(self.mem.GOAL_GRAPH):
               removed_goals.append(goal)

       for goal in removed_goals:
            self.previous_anomolous_goals.remove(goal)

    def parse_tile_to_x_y(self, tile):
        y_index = tile.index('y')
        return int(tile[2:y_index]) , int(tile[y_index+1:])

    def find_direction(self, agent_pos, previous_goal):
        """
        :param agent_pos: Tx1y1
        :param previous_goal: surveyed(something)
        :return:
        """
        agent_pos = self.parse_tile_to_x_y(agent_pos)
        dest_pos = self.parse_tile_to_x_y(previous_goal.args[1])
        if agent_pos[1] > dest_pos[1]:
            return "south"
        elif agent_pos[1] < dest_pos[1]:
            return "north"
        elif agent_pos[0] > dest_pos[0]:
            return "west"
        elif agent_pos[0] < dest_pos[0]:
            return "east"
        else:
            return None

    def find_better(self, direction, agent_pos, reselected_goal):
        """

        :param direction: "north" , "south", "east", "west"
        :param agent_pos: Tx1y1
        :param reselected_goal: surveyed(grace, Tx2y2)
        :return: goal (if reselected goal is not affected by anomaly) or None
        """
        pos = self.parse_tile_to_x_y(agent_pos)
        dest_pos = self.parse_tile_to_x_y(reselected_goal.args[1])

        if direction == "north":
            if dest_pos[1] > pos[1]:
                return None
        elif direction == "south":
            if dest_pos[1] < pos[1]:
                return None
        elif direction == "east":
            if dest_pos[0] > pos[0] and  (dest_pos[1] == pos[1]):
                return None
        elif direction == "west":
            if dest_pos[0] < pos[0] and (dest_pos[1] == pos[1]):
                return None

        return reselected_goal



    def run(self, cycle, verbose=2):
        trace = self.mem.trace
        # reselected goals
        goals = None
        reselected_goals = self.mem.get(self.mem.CURRENT_GOALS)
        formulated_goals = self.mem.get(self.mem.FORMULATED_GOALS)
        current_goals = self.mem.get(self.mem.SUSPENDED_GOALS)
        reselected_goal = None
        formulated_goal = None
        current_goal = None
        if not (reselected_goals and formulated_goals):
            return
        else:
            reselected_goal = reselected_goals[-1][0]
            formulated_goal = formulated_goals[-1][0]
            current_goal = current_goals[-1][0]

            self.mem.set(self.mem.FORMULATED_GOALS, [])
            self.mem.set(self.mem.SUSPENDED_GOALS, [])

        # if anomaly occurs when free and survey ergodic ignore
        if not reselected_goal["predicate"].startswith("surveyed") or \
            reselected_goal["predicate"] == "surveyed-ergodic":
            return

        if current_goal["predicate"] == "surveyed-ergodic":
            return

        # add current goals to previous anomolous goals
        self.remove_anomolous_goals()
        self.set_anomolous_goals(current_goal)
        # find agent position
        world = self.mem.get(self.mem.STATES)[-1]
        agent_position = world.get_atoms(["agent-at", "grace"])[0].args[1].name

        # find anomolous direction
        direction = self.find_direction(agent_position, current_goal)

        # if it's the same cell there is no anomaly; we captured it when it is over
        if direction == None:
            if self.previous_goal:
                self.previous_goal = None
                return None
            else:
                goals = [current_goal]
                self.previous_goal = current_goal
        else:

            # check if reselcted goal is suitable for last 3 directions
            goal = None
            for anamolous_goal in self.previous_anomolous_goals:
                direction = self.find_direction(agent_position, anamolous_goal)
                goal = self.find_better(direction, agent_position, reselected_goal)
                if not goal:
                    break
            # give priority to reselected goal
            if goal:
                #current_goal.status = True
                return None

            #give priority to formulated goal
            else:
                goals = [formulated_goal]
                if not formulated_goal == current_goal:
                    current_goal.status = True

        self.mem.set(self.mem.FORMULATED_GOALS, [])
        self.mem.set(self.mem.SUSPENDED_GOALS, [])

        # current goals as a stack
        if self.mem.get(self.mem.CURRENT_GOALS):
            current_goals = self.mem.get(self.mem.CURRENT_GOALS)
            if not current_goals[-1] == goals:
                current_goals.append(goals)
                self.mem.set(self.mem.CURRENT_GOALS, current_goals)
        else:
            self.mem.set(self.mem.CURRENT_GOALS, [goals])

        if trace:
            trace.add_data("GOALS", goals)

        if not goals:
            if verbose >= 2:
                print "No goals selected."
        else:
            if verbose >= 2:
                print "Selecting goal(s):",
                for goal in goals:
                    print goal,
                print


class HGNSelection(base.BaseModule):

    def __init__(self):
        random.seed(100)

    def subgoals(self, world, goal):
        """
        :param world: world object
        :param goal: current goal
        :return: return possible sub goals
        """

        predicate = goal["predicate"]
        node = world.cltree.check_in_all_tree_nodes(predicate)
        if node:
            return [str(children) for children in node.children]
        else:
            return None

    def hasHGN(self, world, goal):
        """
        :param world: world object
        :param goal: current goal
        :return: return whether the goal has an associated HGN or not
        """
        predicate = goal["predicate"]
        node = world.cltree.check_in_all_tree_nodes(predicate)
        if node:
            return True
        else:
            return False

    def get_time_remaining(self, world):
        """

        :param world: current states
        :return: return time from the states
        """

        time_atom = world.get_atoms(["timeRemaining"])
        # returns the time atom in a list
        if time_atom:
            time = float( time_atom[0].args[0].name)
            return time

    def resource_estimator(self, time, goals):
        """
        :param time: time required to achieve the goals in the goal graph
        :param goals: goals in the goal graph
        :return: a boolean: if there are enough resources to achieve the goals or not
        """

        number_of_goals = len(goals)
        # assume each goal might take around 18 sec
        if time > (600*10)/100:
            return True
        else:
            return False


    def check_corner_cells(self, goal):
        """
        :param goal: goal
        :return: if the goals is to survey corner cell
        """

        # survey goal is of the format surveyed(agent, Tx0y0)
        # To be a corner cell agent should be at Tx0y0, Tx0y4, Tx4y0, Tx4,y4
        corner_cells = ["Tx0y0", "Tx0y4", "Tx4y0", "Tx4y4"]
        position = goal.args[1]
        if position in corner_cells:
            return True
        else:
            return False


    def check_inner_cells(self, goal):
        """
        :param goal: goal
        :return: if the goals is to survey inner cell
        """

        # survey goal is of the format surveyed(agent, Tx0y0)
        # To be a inner cell agent should be at
        # Tx1y3, Tx2y3, Tx3y3,
        # Tx1y2, Tx2y2, Tx3y2,
        # Tx1y1, Tx2y1, Tx3y1,
        inner_cells = ["Tx1y3", "Tx2y3", "Tx3y3",
                       "Tx1y2", "Tx2y2", "Tx3y2",
                       "Tx1y1", "Tx2y1", "Tx3y1"]

        position = goal.args[1]
        if position in inner_cells:
            return True
        else:
            return False


    def rules(self, resources_flag, current_goals, world):
        """
        :param resources_flag: indicates if there are enough resources or not
        :param current_goals: current goals agent should pursue
        :param world: current states
        :return: the goal an agent should pursue
        """
        # Rules: if there are enough resources, then get the subgoals
        #           1. if it is the corner cell then use surveyed_structured
        #           2. If it is an inner cell then use surveyed-singleCellErgodic
        #           3. for remaining use any cell
        #       if there are not enough resources, then get its quivalent goal and
        #           1. perform surveyed-ergodic

        modified_goals = []

        for goal in current_goals:
            # check if the goal has an associated HGN
            if self.hasHGN(world, goal):

                if resources_flag:
                    # if it is the corner cell goal
                    if self.check_corner_cells(goal):
                        if not goal["predicate"] == "surveyed-structured":
                            goal["predicate"] = "surveyed-structured"
                    elif self.check_inner_cells(goal):
                        if not goal["predicate"] == "surveyed-singleCellErgodic":
                            goal["predicate"] = "surveyed-singleCellErgodic"
                    else:
                        sub_goals = self.subgoals(world, goal)
                        if sub_goals:
                            if not goal["predicate"] in sub_goals:
                                goal["predicate"] = "surveyed-structured"
                else:
                    goal["predicate"] = "surveyed-ergodic"
                    if len(goal.args) >  1:
                        goal.args = goal.args[:-1]

            modified_goals.append(goal)

        return modified_goals


    def run(self, cycle, verbose=2):
        world = self.mem.get(self.mem.STATES)[-1]
        world.cltree.printtree()
        trace = self.mem.trace
        if trace:
            trace.add_module(cycle, self.__class__.__name__)
            trace.add_data("GOALGRAPH", copy.deepcopy(self.mem.GOAL_GRAPH))

        goalGraph = self.mem.get(self.mem.GOAL_GRAPH)

        if not goalGraph:
            if verbose >= 1:
                print "Goal graph not initialized. Intend will do nothing."
            return

        current_goals = self.mem.get(self.mem.CURRENT_GOALS)

        if not current_goals:
            return
        else:
            current_goals = current_goals[-1]
            # get all the goals from the root of the goal graph
            goals = goalGraph.getUnrestrictedGoals()

            # calculate resources
            timeRemaining = self.get_time_remaining(world)
            sufficient_resources = self.resource_estimator(timeRemaining, goals)
            # find sub goals and replace them with the rules
            modified_current_goals = self.rules(sufficient_resources, current_goals, world)

        # current goals as a stack
        if self.mem.get(self.mem.CURRENT_GOALS):
            current_goals = self.mem.get(self.mem.CURRENT_GOALS)
            if not current_goals[-1] == modified_current_goals:
                current_goals[-1] = modified_current_goals
                self.mem.set(self.mem.CURRENT_GOALS, current_goals)
        else:
            self.mem.set(self.mem.CURRENT_GOALS, [modified_current_goals])

        if trace:
            trace.add_data("GOALS", modified_current_goals)

        if not modified_current_goals:
            if verbose >= 2:
                print "No goals selected."
        else:
            if verbose >= 2:
                print "Selecting goal(s):",
                for goal in modified_current_goals:
                    print goal,
                print


class SimpleIntendMultipleGoalsSuspend(base.BaseModule):

    def run(self, cycle, verbose = 2):
        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("GOALGRAPH",copy.deepcopy(self.mem.GOAL_GRAPH))

        goalGraph = self.mem.get(self.mem.GOAL_GRAPH)

        if not goalGraph:
            if verbose >= 1:
                print "Goal graph not initialized. Intend will do nothing."
            return
        # get all the goals from the root of the goal graph
        goals = goalGraph.getUnrestrictedGoals()

        if not goals:
            if verbose >= 1:
                print "No Goals in Goal graph. Intend will do nothing."
            return

        world = self.mem.get(self.mem.STATES)[-1]
        emergency = world.get_atoms(["agent-mode","grace", "m3"])
        if emergency:
            # take the first goal
            #goals = [goals[0]]
            # add it to the current goal in memory
            predicates = ["committed", "rejected", "requested"]
            suspended_goal = self.mem.get(self.mem.SUSPENDED_GOALS)
            if not suspended_goal:
                copy_goals = copy.deepcopy(goals)
                suspended_goals = []
                for goal in copy_goals:
                    if not goal['predicate'] in predicates:
                        suspended_goals.append(goal)
                        if verbose >= 2:
                            print "The goal " + str(goal) + " is suspended due to lack of resources"
                        goals.remove(goal)
                self.mem.set(self.mem.SUSPENDED_GOALS, suspended_goals)
            else:
                copy_goals = copy.deepcopy(suspended_goal)
                for goal in copy_goals:
                    if goal in goals:
                        goals.remove(goal)

            # current goals as a stack
            if self.mem.get(self.mem.CURRENT_GOALS) :
                current_goals = self.mem.get(self.mem.CURRENT_GOALS)
                if not current_goals[-1] == goals:
                    current_goals = [goals]
                    self.mem.set(self.mem.CURRENT_GOALS, current_goals)
                else:
                    goals = []
            else:
                self.mem.set(self.mem.CURRENT_GOALS, [goals])

            if trace:
                trace.add_data("GOALS",goals)

            if not goals:
                if verbose >= 2:
                    print "No goals selected."
            else:
                if verbose >= 2:
                    print "Selecting goal(s):",
                    for goal in goals:
                        print goal,
                    print
