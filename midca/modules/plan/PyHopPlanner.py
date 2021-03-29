from midca.modules._plan import pyhop
from midca.modules._plan import modified_pyhop
from midca import plans, base
from midca.modules._plan.asynch import asynch
from midca.modules._plan.jShop import JSHOP, JSHOP2
from midca.modules._plan.pyhop import print_state,  print_methods, print_operators
from midca.modules.plan import GenericPyhopPlanner
import collections
import traceback
import copy
import time
import itertools

class PyHopPlanner(base.BaseModule):

    '''
    MIDCA module that implements a python version of the SHOP hierarchical task network (HTN) planner. HTN planners require a set of user-defined methods to generate plans; these are defined in the methods python module and declared in the constructor for this class.
    Note that this module uses has several methods to translate between MIDCA's world and goal representations and those used by pyhop; these should be changed if a new domain is introduced.
    '''

    pyhop_state_from_world = None
    pyhop_tasks_from_goals = None

    def __init__(self,
                 pyhop_state_from_world,
                 pyhop_tasks_from_goals,
                 declare_methods,
                 declare_operators,
                 extinguishers = False,
                 mortar = False):

        self.pyhop_state_from_world = pyhop_state_from_world
        self.pyhop_tasks_from_goals = pyhop_tasks_from_goals

        try:
            declare_methods()
            declare_operators()
            self.working = True
        except:
            print("Error declaring pyhop methods and operators. This planner will be \
            disabled")
            traceback.print_exc()
            self.working = False

    def init(self, world, mem):
        self.world = world
        self.mem = mem
        self.mem.set(self.mem.PLANNING_COUNT, 0)


    #this will require a lot more error handling, but ignoring now for debugging.
    def run(self, cycle, verbose = 2):
        world = self.mem.get(self.mem.STATES)[-1]
        try:
            goals = self.mem.get(self.mem.CURRENT_GOALS)[-1]
        except:
            goals = []
        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("WORLD", copy.deepcopy(world))
            trace.add_data("GOALS", copy.deepcopy(goals))

        if not goals:
            if verbose >= 2:
                print("No goals received by planner. Skipping planning.")
            return
        try:
            midcaPlan = self.mem.get(self.mem.GOAL_GRAPH).getMatchingPlan(goals)
        except AttributeError:
            midcaPlan = None
        if midcaPlan:
            if verbose >= 2:
                print("Old plan retrieved. Checking validity...", end=' ')
            valid = world.plan_correct(midcaPlan)
            if not valid:
                midcaPlan = None
                #if plan modification is added to MIDCA, do it here.
                if verbose >= 2:
                    print("invalid.")
            elif verbose >= 2:
                print("valid.")
            if valid:
                if verbose >= 2:
                    print("checking to see if all goals are achieved...", end=' ')
                achieved = world.plan_goals_achieved(midcaPlan)
                if verbose >= 2:
                    if len(achieved) == len(midcaPlan.goals):
                        print("yes")
                    else:
                        print("no. Goals achieved: " + str({str(goal) for goal in achieved}))
                if len(achieved) != len(midcaPlan.goals):
                    midcaPlan = None #triggers replanning.

        #ensure goals is a collection to simplify things later.
        if not isinstance(goals, collections.Iterable):
            goals = [goals]

        if not midcaPlan:
            #use pyhop to generate new plan
            if verbose >= 2:
                print("Planning...")
            try:
                pyhopState = self.pyhop_state_from_world(world)
            except Exception:
                print("Could not generate a valid pyhop state from current world state. Skipping planning")

            try:
                pyhopTasks = self.pyhop_tasks_from_goals(goals,pyhopState)
            except Exception:
                print("Could not generate a valid pyhop task from current goal set. Skipping planning")
            try:
                #print_state(pyhopState)
                # record attempt to replann
                self.mem.set(self.mem.PLANNING_COUNT, 1+self.mem.get(self.mem.PLANNING_COUNT))
                pyhopPlan = pyhop.pyhop(pyhopState, pyhopTasks, verbose = 0)
            except Exception:

                pyhopPlan = None
            if not pyhopPlan and pyhopPlan != []:
                if verbose >= 1:
                    print("Planning failed for ", end=' ')
                    for goal in goals:
                        print(goal, " ", end=' ')
                    print()

                if trace: trace.add_data("PLAN", None) # planning failed, record NONE for plan
                return
            #change from pyhop plan to MIDCA plan
            midcaPlan = plans.Plan([plans.Action(action[0], *list(action[1:])) for action in pyhopPlan], goals)

            if verbose >= 1:
                print("Planning complete.")
            if verbose >= 2:
                print("Plan: ")#, midcaPlan
                for a in midcaPlan:
                    print(("  "+str(a)))
            #save new plan
            if midcaPlan != None:
                self.mem.get(self.mem.GOAL_GRAPH).addPlan(midcaPlan)
            if trace: trace.add_data("PLAN",midcaPlan)

class PyHopPlanner_temporary(base.BaseModule):

    '''
    MIDCA module that implements a python version of the SHOP hierarchical task network (HTN) planner. HTN planners require a set of user-defined methods to generate plans; these are defined in the methods python module and declared in the constructor for this class.
    Note that this module uses has several methods to translate between MIDCA's world and goal representations and those used by pyhop; these should be changed if a new domain is introduced.
    '''

    pyhop_state_from_world = None
    pyhop_tasks_from_goals = None

    def __init__(self,
                 pyhop_state_from_world,
                 pyhop_tasks_from_goals,
                 declare_methods,
                 declare_operators,
                 extinguishers = False,
                 mortar = False):

        self.pyhop_state_from_world = pyhop_state_from_world
        self.pyhop_tasks_from_goals = pyhop_tasks_from_goals

        try:
            declare_methods()
            declare_operators()
            self.working = True
        except:
            print("Error declaring pyhop methods and operators. This planner will be \
            disabled")
            traceback.print_exc()
            self.working = False

    def init(self, world, mem):
        self.world = world
        self.mem = mem
        self.mem.set(self.mem.PLANNING_COUNT, 0)


    #this will require a lot more error handling, but ignoring now for debugging.
    def run(self, cycle, verbose = 2):
        world = self.mem.get(self.mem.STATES)[-1]
        try:
            goals = self.mem.get(self.mem.CURRENT_GOALS)
        except:
            goals = []
        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("WORLD", copy.deepcopy(world))
            trace.add_data("GOALS", copy.deepcopy(goals))

        if not goals:
            if verbose >= 2:
                print("No goals received by planner. Skipping planning.")
            return
        try:
            midcaPlan = self.mem.get(self.mem.GOAL_GRAPH).getMatchingPlan(goals)
        except AttributeError:
            midcaPlan = None
        if midcaPlan:
            if verbose >= 2:
                print("Old plan retrieved. Checking validity...", end=' ')
            valid = world.plan_correct(midcaPlan)
            if not valid:
                midcaPlan = None
                #if plan modification is added to MIDCA, do it here.
                if verbose >= 2:
                    print("invalid.")
            elif verbose >= 2:
                print("valid.")
            if valid:
                if verbose >= 2:
                    print("checking to see if all goals are achieved...", end=' ')
                achieved = world.plan_goals_achieved(midcaPlan)
                if verbose >= 2:
                    if len(achieved) == len(midcaPlan.goals):
                        print("yes")
                    else:
                        print("no. Goals achieved: " + str({str(goal) for goal in achieved}))
                if len(achieved) != len(midcaPlan.goals):
                    midcaPlan = None #triggers replanning.

        #ensure goals is a collection to simplify things later.
        if not isinstance(goals, collections.Iterable):
            goals = [goals]

        if not midcaPlan:
            #use pyhop to generate new plan
            if verbose >= 2:
                print("Planning...")
            try:
                pyhopState = self.pyhop_state_from_world(world)
            except Exception:
                print("Could not generate a valid pyhop state from current world state. Skipping planning")

            try:
                pyhopTasks = self.pyhop_tasks_from_goals(goals,pyhopState)
            except Exception:
                print("Could not generate a valid pyhop task from current goal set. Skipping planning")
            try:
                #print_state(pyhopState)
                # record attempt to replann
                self.mem.set(self.mem.PLANNING_COUNT, 1+self.mem.get(self.mem.PLANNING_COUNT))
                pyhopPlan = pyhop.pyhop(pyhopState, pyhopTasks, verbose = 0)
            except Exception:

                pyhopPlan = None
            if not pyhopPlan and pyhopPlan != []:
                if verbose >= 1:
                    print("Planning failed for ", end=' ')
                    for goal in goals:
                        print(goal, " ", end=' ')
                    print()

                if trace: trace.add_data("PLAN", None) # planning failed, record NONE for plan
                return
            #change from pyhop plan to MIDCA plan
            midcaPlan = plans.Plan([plans.Action(action[0], *list(action[1:])) for action in pyhopPlan], goals)

            if verbose >= 1:
                print("Planning complete.")
            if verbose >= 2:
                print("Plan: ")#, midcaPlan
                for a in midcaPlan:
                    print(("  "+str(a)))
            #save new plan
            if midcaPlan != None:
                self.mem.get(self.mem.GOAL_GRAPH).addPlan(midcaPlan)
            if trace: trace.add_data("PLAN",midcaPlan)
