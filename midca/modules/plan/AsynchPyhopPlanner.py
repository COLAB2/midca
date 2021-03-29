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

class AsynchPyhopPlanner(GenericPyhopPlanner):

    '''
    This planner is the same as the GenericPyhopPlanner, but it returns an asynchronous
    plan.
    '''

    def __init__(self, declare_methods, declare_operators,declare_monitors):
        GenericPyhopPlanner.__init__(self, declare_methods,declare_operators, declare_monitors,
        lambda state, plan: asynch.FAILED not in [action.status for action in plan])

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
        if verbose > 2:
            if plan:
                print("Will not replan")
            else:
                print("Will replan")
        if plan:
            return
        if not plan:
            # 1 specifies to monitor
            plan = self.get_new_modified_plan(state, goals, verbose)
        if not plan:
            return
        midcaPlan = plans.Plan(plan, goals)
        asynchPlan = asynch.asynch_plan(self.mem, midcaPlan)
        if verbose >= 1:
            print("Planning complete.")
            if verbose >= 2:
                print("Plan: ", asynchPlan)
        #save new plan
        if asynchPlan != None:
            self.mem.get(self.mem.GOAL_GRAPH).addPlan(asynchPlan)

class AsynchPyhopPlanner_3d_camera(GenericPyhopPlanner):

    '''
    This planner is the same as the GenericPyhopPlanner, but it returns an asynchronous
    plan.
    '''

    def __init__(self, declare_methods, declare_operators,declare_monitors):
        GenericPyhopPlanner.__init__(self, declare_methods,declare_operators,declare_monitors,
        lambda state, plan: asynch.FAILED not in [action.status for action in plan])


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
        if verbose > 2:
            if plan:
                print("Will not replan")
            else:
                print("Will replan")
        if plan:
            return
        if not plan:
            plan = self.get_new_modified_plan(state, goals, verbose)
        if not plan:
            return
        midcaPlan = plans.Plan(plan, goals)
        asynchPlan = asynch_3d.asynch_plan(self.mem, midcaPlan)
        if verbose >= 1:
            print("Planning complete.")
            if verbose >= 2:
                print("Plan: ", asynchPlan)
        #save new plan
        if asynchPlan != None:
            self.mem.get(self.mem.GOAL_GRAPH).addPlan(asynchPlan)
