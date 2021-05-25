from midca import base,midcatime
import copy,itertools,operator
import random
import time as ctime


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
                print("Goal graph not initialized. Intend will do nothing.")
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
                print("No goals selected.")
        else:
            if verbose >= 2:
                print("Selecting goal(s):", end=' ')
                for goal in goals:
                    print(goal, end=' ')
                print()

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
                print("No Goals in Goal graph. Intend will do nothing.")
                return

        if not goalGraph:
            if verbose >= 1:
                print("Goal graph not initialized. Intend will do nothing.")
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
                        print((self.mem.get(self.mem.EXECUTED_BUILDING_LIST)))
                    else:
                        self.mem.set(self.mem.EXECUTED_BUILDING_LIST , [len(self.mem.get(self.mem.CURRENT_GOALS))])
                        print((self.mem.get(self.mem.EXECUTED_BUILDING_LIST)))



                # if zone is -1, then re- initialize self.count to 0
                # if there is no sufficient time, then give a hint to eval to remove all goals
                zone = 0
                if not (self.count == i):
                    random.seed(i)
                    action_score = random.uniform(1 - time_variation*1,1 + time_variation*1)
                    action_time = random.uniform(self.t_copy[i] - time_variation*self.t_copy[i], self.t_copy[i] + time_variation*self.t_copy[i])
                    print(("Actual Time Taken For Previous Action :" + str(action_time)))
                    print(("Expected Time Taken For Previous Action :" + str(self.t_copy[i])))
                    print(("Actual Score for Previous Action : " + str(action_score)))

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
                            if self.implement == "selection":
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




        print(("Time Remaining : " + str(self.T)))



        if ( (ctime.time() - self.time) > 120) and (self.T >= 0):
            self.time = ctime.time()
            self.T = random.uniform(0.7*(self.T), self.T)
            #self.time_change = True
            print("")
            print(("Time is changed & Time Remaining is: " + str(self.T)))
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
                print(("Processing" + str(self.mem.get(self.mem.CURRENT_GOALS))))
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
            print(("Tower " + self.building_list[i][0].args[0]))
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


        if self.implement == "stack":
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
                print(("Selected Towers are : " + str(towers)))
                print("")
                length = [len(each) for each in self.selected_buildings]
                if not self.mem.get(self.mem.SELECTED_BUILDING_LIST):
                    self.mem.set(self.mem.SELECTED_BUILDING_LIST , length)
                print(("Overall Score : " + str(score)))
                print(("Overall Time is : " + str(time)))
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
            print(("Selected Towers are : " + str(towers)))
            print("")

        # when there is  time limit
        else:
            self.selected_buildings = []
            a = []
            for i in range(0,len(self.buildings_scores['t'])):
                a.append(i)
            for i in range(1,len(a)+1):
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
            sorted_x = sorted(list(self.combinations_t.items()), key=operator.itemgetter(1))

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


            sorted_x = sorted(list(self.combinations_b.items()), key=operator.itemgetter(1))

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
            print(("Selected Towers are : " + str(towers)))
            print(("Overall Score : " + str(score)))
            print(("Overall Time is : " + str(time)))
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
                print("No goals selected.")
        else:
            if verbose >= 2:
                print(("Time is " + str(self.T)))
                print("")
                print("Selecting Tower(s):", end=' ')
                #print("")


                for i in range(0,len(self.selected_buildings)):
                    print("")
                    print(("Tower " + self.selected_buildings[i][0].args[0] ), end=' ')
                    if(len(self.selected_buildings[i]) == 1):
                        print(("(" + str(len(self.selected_buildings[i])) + " Block)"), end=' ')
                    else:
                        print(("(" + str(len(self.selected_buildings[i])) + " Blocks)"), end=' ')
                    print((" with priority : " + str(i+1)), end=' ')
                    print(("["), end=' ')
                    #for goals in self.selected_buildings[i]:
                    #       print(goals)
                    #print("]")
                    #print("")
                    print(("Time(T) :" + str(self.t[len(self.selected_buildings[i])-1]) + ","), end=' ')
                    print(("Score(P) :" + str(len(self.selected_buildings[i])) + ","), end=' ')
                    print(("P/T :" + str(len(self.selected_buildings[i])/self.t[len(self.selected_buildings[i])-1]) + " ]"), end=' ')
                print("")


                self.goals = copy.deepcopy(self.selected_buildings[0])
                del self.selected_buildings[0]

                '''
                for goal in self.selected_goals:
                    print goal,
                print
                '''
                print("")
