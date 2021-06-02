import copy
from midca.modules.interpret._adist import ADistance, ChangeFinder, WindowPair, Interval
from midca.domains.nbeacons import nbeacons_util
import sys

class GoalDiscrepancyDetector:
    '''
    Performs Goal Discrepancy Detection using Informed Expectations
    TODO: This is a stub for now, not implemented yet
    '''

    def __init__(self, world_planner):
        self.world_planner = world_planner
        pass


    def init(self, world, mem):
        self.world = world
        self.mem = mem

    def run(self, cycle, verbose=2):

        trace = self.mem.trace
        if trace:
            trace.add_module(cycle, self.__class__.__name__)
            trace.add_data("GOALGRAPH", copy.deepcopy(self.mem.GOAL_GRAPH))


        
        prev_action = self.mem.get(self.mem.ACTIONS)
        curr_goals = self.mem.get(self.mem.CURRENT_GOALS)
        list_goal=[]
        if self.mem.get(self.mem.Goal_initial):

            print(self.mem.get(self.mem.Goal_initial))

            pass
        else:
            self.mem.set(self.mem.Goal_initial,copy.deepcopy(curr_goals))
        #a=1
        if(not self.mem.GOAL_LIST):
            try:
                for goal in curr_goals[-1]:
                    achieved = self.world.atom_true(self.world.midcaGoalAsAtom(goal))
                    if 'negate' in goal and goal['negate']:
                        achieved = not achieved
                    list_goal.append(achieved)
                    self.mem.GOAL_LIST = list_goal
            except:
                pass


        if(prev_action is None):
            pass
        else:
            world_planner = self.world_planner.copy()
            copy_world = self.mem.get(self.mem.STATES)[-2].copy()
            last_actions = self.mem.get(self.mem.ACTIONS)[-1]
            try:
                last_action=last_actions[0]
            except:
                return

            goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
            try:
                goals1 = self.mem.get(self.mem.CURRENT_GOALS)[-1]
            except:
                goals1 = []
            plans = goalGraph.allMatchingPlans(goals1)
            try:
                plans = plans[0]
            except:
                return

            for atom in self.world_planner.atoms:
                world_planner.remove_atom(atom)

            for atom in copy_world.atoms:
                world_planner.add_atom(atom)

            curr_list_goal = []
            name_goal=[]
            #adding the current goal achieved in a list
            for goal in curr_goals[-1]:
                achieved = self.world.atom_true(self.world.midcaGoalAsAtom(goal))
                name_goal.append(self.world.midcaGoalAsAtom(goal))
                if 'negate' in goal and goal['negate']:
                    achieved = not achieved
                curr_list_goal.append(achieved)
            k=[]
            self.mem.WORLD_CHANGES=[]
            #checking the previous goals achieved with the current goals list
            for i in range(len(curr_list_goal)):

                if (curr_list_goal[i] is False and self.mem.GOAL_LIST[i] is True) :
                    print("discrepancyyyy")
                    self.mem.WORLD_CHANGES.append(name_goal[i])
                    goalGraph.remove(goals1[i])
                    k.append(i)
                        #print(goalGraph)
                    self.mem.DISCREPANCY = True
                else:
                    pass
            try:
                for i in k:
                    curr_list_goal.remove(curr_list_goal[i])
            except:
                pass

            self.mem.GOAL_LIST = curr_list_goal

            curr_goal_sets=[]

            try:
                #checking for the rebellion condition before performing spray
                if(self.mem.DISCREPANCY is True and str(plans.get_next_step())[:5] == "spray"):
                    a = str(plans.get_next_step()).split(', ')
                    if (a[0][6] == '('):
                        self.mem.LOCATION = a[0][7:]
                    else:
                        self.mem.LOCATION = a[0][6:]
                    self.mem.GOAL_NEEDED = goals1
                    for x in self.world.atoms:
                        for i in range(1,len(a)):
                            if(str(x) == "native-at({})".format(a[i]) ):
                                print("about to kill native plants")
                                predicateName = "knows"
                                args = ['human1','not-invasive-at',self.mem.LOCATION]
                                curr_goal_sets.append(goals.Goal(*args, predicate=predicateName))
                                for g in curr_goal_sets:
                                    self.mem.get(self.mem.GOAL_GRAPH).insert(g)
            except:
                return
