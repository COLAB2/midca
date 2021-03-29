from midca.modules._plan import pyhop
from midca.modules._plan import modified_pyhop
from midca import plans, base
from midca.modules._plan.asynch import asynch
from midca.modules._plan.jShop import JSHOP, JSHOP2
from midca.modules._plan.pyhop import print_state,  print_methods, print_operators
import collections
import traceback
import copy
import time
import itertools

class GenericPyhopPlanner(base.BaseModule):

    '''
    Whereas the PyHopPlanner class below is optimized for use with MIDCA's built-in world
    simulator, this planner is more generalized. It assumes that the world state stored
    in MIDCA's memory is also the world state that will be expected by the planning
    methods and operators. Also, it expects to receive 'declare_methods' and
    'declare_operators' methods as arguments. These should initialize pyhop for the
    desired planning domain. The plan_validator arg should be a method which takes a
    world state and a plan as args and returns whether the plan should be used. This will
    only be called on old plans that are retrieved.
    '''

    def __init__(self, declare_methods, declare_operators, declare_monitors=None, plan_validator = None):
        try:
            declare_methods()
            declare_operators()
            if declare_monitors:
                declare_monitors()
            self.working = True
        except:
            print("Error declaring pyhop methods and operators. This planner will be \
            disabled")
            traceback.print_exc()
            self.working = False
        self.validate_plan = plan_validator
                #note by default (no plan validator)
        #plans execute to completion unless goals change

    def get_old_plan(self, state, goals, verbose = 2):
        try:
            plan = self.mem.get(self.mem.GOAL_GRAPH).getMatchingPlan(goals)
            if not plan:
                return None
            try:
                if self.validate_plan:
                    valid = self.validate_plan(state, plan)
                    if valid:
                        if verbose >= 2:
                            print("Old plan found that tests as valid:", plan)
                    else:
                        if verbose >= 2:
                            print("Old plan found that tests as invalid:", plan, ". removing from stored plans.")
                        self.mem.get(self.mem.GOAL_GRAPH).removePlan(plan)
                else:
                    if verbose >= 2:
                        print("no validity check specified. assuming old plan is valid.")
                        valid = True
            except:
                if verbose >= 2:
                    print("Error validating plan:", plan)
                valid = False
        except:
            print("Error checking for old plans")
            plan = None
            valid = False
        if valid:
            return plan
        return None

    def get_new_plan(self, state, goals, verbose = 2):
        '''
            Calls the pyhop planner to generate a new plan.
        '''

        if verbose >= 2:
            print("Planning...")
        try:
            plan = pyhop.pyhop(state, [("achieve_goals", goals)], verbose = 0)
            #note: MIDCA does not convert its state and goals to pyhop state and
            #goal objects. Therefore, pyhop will not print correctly if verbose is
            #set to other than 0.
        except:
            if verbose >= 1:
                print("Error in planning:", traceback.format_exc(), "\n-Planning failed.")
            return None
        return plan

    def get_new_modified_plan(self, state, goals, verbose = 2):
        '''
            Calls the pyhop planner to generate a new plan.
        '''

        if verbose >= 2:
            print("Planning...")
        try:
            plan = modified_pyhop.pyhop(state, [("achieve_goals", goals)], verbose = 0)
            #note: MIDCA does not convert its state and goals to pyhop state and
            #goal objects. Therefore, pyhop will not print correctly if verbose is
            #set to other than 0.
        except:
            if verbose >= 1:
                print("Error in planning:", traceback.format_exc(), "\n-Planning failed.")
            return None
        return plan

    def run(self, cycle, verbose = 2):
        state = self.mem.get(self.mem.STATE)
        if not state:
            states  = self.mem.get(self.mem.STATES)
            if states:
                state = states[-1]
            else:
                if verbose >= 1:
                    print("No world state loaded. Skipping planning.")
                return
        #now state is the most recent (or only) state and is non-null
        try:
            goals = self.mem.get(self.mem.CURRENT_GOALS)[-1]
        except:
            goals = []
        if not goals:
            if verbose >= 2:
                print("No goals received by planner. Skipping planning.")
            return
        plan = self.get_old_plan(state, goals, verbose)
        if verbose >= 2:
            if plan:
                print("Will not replan")
            else:
                print("Planning from scratch")
        if not plan:
            plan = self.get_new_plan(state, goals, verbose)
            if not plan and plan != []:
                return
            #convert to MIDCA plan format
            plan = plans.Plan(
                              [plans.Action(action[0], *action[1:]) for
                               action in plan], goals)
            if verbose >= 1:
                print("Planning complete.")
        if verbose >= 2:
            print("Plan: ", plan)
        #save new plan
        if plan != None:
            self.mem.get(self.mem.GOAL_GRAPH).addPlan(plan)

