from _plan import pyhop
from _plan import modified_pyhop
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
            print "Error declaring pyhop methods and operators. This planner will be \
            disabled"
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
                            print "Old plan found that tests as valid:", plan
                    else:
                        if verbose >= 2:
                            print "Old plan found that tests as invalid:", plan, ". removing from stored plans."
                        self.mem.get(self.mem.GOAL_GRAPH).removePlan(plan)
                else:
                    if verbose >= 2:
                        print "no validity check specified. assuming old plan is valid."
                        valid = True
            except:
                if verbose >= 2:
                    print "Error validating plan:", plan
                valid = False
        except:
            print "Error checking for old plans"
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
            print "Planning..."
        try:
            plan = pyhop.pyhop(state, [("achieve_goals", goals)], verbose = 0)
            #note: MIDCA does not convert its state and goals to pyhop state and
            #goal objects. Therefore, pyhop will not print correctly if verbose is
            #set to other than 0.
        except:
            if verbose >= 1:
                print "Error in planning:", traceback.format_exc(), "\n-Planning failed."
            return None
        return plan

    def get_new_modified_plan(self, state, goals, verbose = 2):
        '''
            Calls the pyhop planner to generate a new plan.
        '''

        if verbose >= 2:
            print "Planning..."
        try:
            plan = modified_pyhop.pyhop(state, [("achieve_goals", goals)], verbose = 0)
            #note: MIDCA does not convert its state and goals to pyhop state and
            #goal objects. Therefore, pyhop will not print correctly if verbose is
            #set to other than 0.
        except:
            if verbose >= 1:
                print "Error in planning:", traceback.format_exc(), "\n-Planning failed."
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
                    print "No world state loaded. Skipping planning."
                return
        #now state is the most recent (or only) state and is non-null
       	try:
            goals = self.mem.get(self.mem.CURRENT_GOALS)[-1]
        except:
            goals = []
        if not goals:
            if verbose >= 2:
                print "No goals received by planner. Skipping planning."
            return
        plan = self.get_old_plan(state, goals, verbose)
        if verbose >= 2:
            if plan:
                print "Will not replan"
            else:
                print "Planning from scratch"
        if not plan:
            plan = self.get_new_plan(state, goals, verbose)
            if not plan and plan != []:
                return
            #convert to MIDCA plan format
            plan = plans.Plan(
                              [plans.Action(action[0], *action[1:]) for
                               action in plan], goals)
            if verbose >= 1:
                print "Planning complete."
        if verbose >= 2:
            print "Plan: ", plan
        #save new plan
        if plan != None:
            self.mem.get(self.mem.GOAL_GRAPH).addPlan(plan)

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
                    print "No world state loaded. Skipping planning."
                return
        #now state is the most recent (or only) state and is non-null
	try:
            goals = self.mem.get(self.mem.CURRENT_GOALS)[-1]
        except:
            goals = []
        if not goals:
            if verbose >= 2:
                print "No goals received by planner. Skipping planning."
            return
        plan = self.get_old_plan(state, goals, verbose)
        if verbose > 2:
            if plan:
                print "Will not replan"
            else:
                print "Will replan"
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
            print "Planning complete."
            if verbose >= 2:
                print "Plan: ", asynchPlan
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
                    print "No world state loaded. Skipping planning."
                return
        #now state is the most recent (or only) state and is non-null

	try:
            goals = self.mem.get(self.mem.CURRENT_GOALS)[-1]
        except:
            goals = []
        if not goals:
            if verbose >= 2:
                print "No goals received by planner. Skipping planning."
            return
        plan = self.get_old_plan(state, goals, verbose)
        if verbose > 2:
            if plan:
                print "Will not replan"
            else:
                print "Will replan"
        if plan:
            return
        if not plan:
            	plan = self.get_new_modified_plan(state, goals, verbose)
        if not plan:
            return
        midcaPlan = plans.Plan(plan, goals)
	asynchPlan = asynch_3d.asynch_plan(self.mem, midcaPlan)
        if verbose >= 1:
            print "Planning complete."
            if verbose >= 2:
                print "Plan: ", asynchPlan
        #save new plan
        if asynchPlan != None:
            self.mem.get(self.mem.GOAL_GRAPH).addPlan(asynchPlan)



class JSHOP2Planner(base.BaseModule):
    '''
    MIDCA module that implements a python version of the SHOP hierarchical task network (HTN) planner. HTN planners require a set of user-defined methods to generate plans; these are defined in the methods python module and declared in the constructor for this class.
    Note that this module uses has several methods to translate between MIDCA's world and goal representations and those used by pyhop; these should be changed if a new domain is introduced.
    '''

    jshop_state_from_world = None
    jshop_tasks_from_goals = None
    domain_file = ""
    state_file = ""
    
    def __init__(self,
                 jshop_state_from_world,
                 jshop_tasks_from_goals,
                 domain_file,
                 state_file,
                 extinguishers = False,
                 mortar = False):

        self.jshop_state_from_world = jshop_state_from_world
        self.jshop_tasks_from_goals = jshop_tasks_from_goals
        self.domain_file = domain_file
        self.state_file= state_file

        try:
            self.working = True
        except:
            print "Error declaring pyhop methods and operators. This planner will be \
            disabled"
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
                print "No goals received by planner. Skipping planning."
            return
        try:
            midcaPlan = self.mem.get(self.mem.GOAL_GRAPH).getMatchingPlan(goals)
        except AttributeError:
            midcaPlan = None
        if midcaPlan:
            if verbose >= 2:
                print "Old plan retrieved. Checking validity...",
            valid = world.plan_correct(midcaPlan)
            if not valid:
                midcaPlan = None
                #if plan modification is added to MIDCA, do it here.
                if verbose >= 2:
                    print "invalid."
            elif verbose >= 2:
                print "valid."
            if valid:
                if verbose >= 2:
                    print "checking to see if all goals are achieved...",
                achieved = world.plan_goals_achieved(midcaPlan)
                if verbose >= 2:
                    if len(achieved) == len(midcaPlan.goals):
                        print "yes"
                    else:
                        print "no. Goals achieved: " + str({str(goal) for goal in achieved})
                if len(achieved) != len(midcaPlan.goals):
                    midcaPlan = None #triggers replanning.

        #ensure goals is a collection to simplify things later.
        if not isinstance(goals, collections.Iterable):
            goals = [goals]

        if not midcaPlan:
            #use pyhop to generate new plan
            if verbose >= 2:
                print "Planning..."
#             try:
            jshopState = self.jshop_state_from_world(world, self.state_file)
#             except Exception:
#                 print "Could not generate a valid pyhop state from current world state. Skipping planning"
#             try:
            jshopTasks = self.jshop_tasks_from_goals(goals,jshopState, self.state_file)
#             except Exception:
#                 print "Could not generate a valid pyhop task from current goal set. Skipping planning"
            try:
                self.mem.set(self.mem.PLANNING_COUNT, 1+self.mem.get(self.mem.PLANNING_COUNT))
                jshopPlan = JSHOP2.jshop(jshopTasks, self.domain_file, self.state_file)
            except Exception:
                jshopPlan = None
            if not jshopPlan and jshopPlan != []:
                if verbose >= 1:
                    print "Planning failed for ",
                    for goal in goals:
                        print goal, " ",
                    print
                if trace: trace.add_data("PLAN", jshopPlan)
                return
            #change from jshop plan to MIDCA plan
            midcaPlan = plans.Plan([plans.Action(action[0], *list(action[1:])) for action in jshopPlan], goals)

            if verbose >= 1:
                print "Planning complete."
            if verbose >= 2:
                print "Plan: "#, midcaPlan
                for a in midcaPlan:
                    print("  "+str(a))
            #save new plan
            if midcaPlan != None:
                self.mem.get(self.mem.GOAL_GRAPH).addPlan(midcaPlan)
            if trace: trace.add_data("PLAN",midcaPlan)


class JSHOPPlanner(base.BaseModule):
    '''
    MIDCA module that implements a python version of the SHOP hierarchical task network (HTN) planner. HTN planners require a set of user-defined methods to generate plans; these are defined in the methods python module and declared in the constructor for this class.
    Note that this module uses has several methods to translate between MIDCA's world and goal representations and those used by pyhop; these should be changed if a new domain is introduced.
    '''

    jshop_state_from_world = None
    jshop_tasks_from_goals = None
    domain_file = ""
    state_file = ""
    
    def __init__(self,
                 jshop_state_from_world,
                 jshop_tasks_from_goals,
                 domain_file,
                 state_file,
                 extinguishers = False,
                 mortar = False):

        self.jshop_state_from_world = jshop_state_from_world
        self.jshop_tasks_from_goals = jshop_tasks_from_goals
        self.domain_file = domain_file
        self.state_file= state_file
        
        try:
            self.working = True
        except:
            print "Error declaring pyhop methods and operators. This planner will be \
            disabled"
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
                print "No goals received by planner. Skipping planning."
            return
        try:
            midcaPlan = self.mem.get(self.mem.GOAL_GRAPH).getMatchingPlan(goals)
        except AttributeError:
            midcaPlan = None
        if midcaPlan:
            if verbose >= 2:
                print "Old plan retrieved. Checking validity...",
            valid = world.plan_correct(midcaPlan)
            if not valid:
                midcaPlan = None
                #if plan modification is added to MIDCA, do it here.
                if verbose >= 2:
                    print "invalid."
            elif verbose >= 2:
                print "valid."
            if valid:
                if verbose >= 2:
                    print "checking to see if all goals are achieved...",
                achieved = world.plan_goals_achieved(midcaPlan)
                if verbose >= 2:
                    if len(achieved) == len(midcaPlan.goals):
                        print "yes"
                    else:
                        print "no. Goals achieved: " + str({str(goal) for goal in achieved})
                if len(achieved) != len(midcaPlan.goals):
                    midcaPlan = None #triggers replanning.

        #ensure goals is a collection to simplify things later.
        if not isinstance(goals, collections.Iterable):
            goals = [goals]

        if not midcaPlan:
            #use jshop to generate new plan
            if verbose >= 2:
                print "Planning..."
            try:
                jshopState = self.jshop_state_from_world(world, self.state_file)
            except Exception:
                print "Could not generate a valid jshop state from current world state. Skipping planning"
            try:
                jshopTasks = self.jshop_tasks_from_goals(goals,jshopState, self.state_file)
            except Exception:
                print "Could not generate a valid jshop task from current goal set. Skipping planning"
            try:
                self.mem.set(self.mem.PLANNING_COUNT, 1+self.mem.get(self.mem.PLANNING_COUNT))
                jshopPlan = JSHOP.jshop(jshopTasks, self.domain_file, self.state_file)
            except Exception:
                jshopPlan = None
            if not jshopPlan and jshopPlan != []:
                if verbose >= 1:
                    print "Planning failed for ",
                    for goal in goals:
                        print goal, " ",
                    print
                if trace: trace.add_data("PLAN", jshopPlan)
                return
            #change from jshop plan to MIDCA plan
            midcaPlan = plans.Plan([plans.Action(action[0], *list(action[1:])) for action in jshopPlan], goals)

            if verbose >= 1:
                print "Planning complete."
            if verbose >= 2:
                print "Plan: "#, midcaPlan
                for a in midcaPlan:
                    print("  "+str(a))
            #save new plan
            if midcaPlan != None:
                self.mem.get(self.mem.GOAL_GRAPH).addPlan(midcaPlan)
            if trace: trace.add_data("PLAN",midcaPlan)


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
            print "Error declaring pyhop methods and operators. This planner will be \
            disabled"
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
                print "No goals received by planner. Skipping planning."
            return
        try:
            midcaPlan = self.mem.get(self.mem.GOAL_GRAPH).getMatchingPlan(goals)
        except AttributeError:
            midcaPlan = None
        if midcaPlan:
            if verbose >= 2:
                print "Old plan retrieved. Checking validity...",
            valid = world.plan_correct(midcaPlan)
            if not valid:
                midcaPlan = None
                #if plan modification is added to MIDCA, do it here.
                if verbose >= 2:
                    print "invalid."
            elif verbose >= 2:
                print "valid."
            if valid:
                if verbose >= 2:
                    print "checking to see if all goals are achieved...",
                achieved = world.plan_goals_achieved(midcaPlan)
                if verbose >= 2:
                    if len(achieved) == len(midcaPlan.goals):
                        print "yes"
                    else:
                        print "no. Goals achieved: " + str({str(goal) for goal in achieved})
                if len(achieved) != len(midcaPlan.goals):
                    midcaPlan = None #triggers replanning.

        #ensure goals is a collection to simplify things later.
        if not isinstance(goals, collections.Iterable):
            goals = [goals]

        if not midcaPlan:
            #use pyhop to generate new plan
            if verbose >= 2:
                print "Planning..."
            try:
                pyhopState = self.pyhop_state_from_world(world)
            except Exception:
                print "Could not generate a valid pyhop state from current world state. Skipping planning"

            try:
                pyhopTasks = self.pyhop_tasks_from_goals(goals,pyhopState)
            except Exception:
                print "Could not generate a valid pyhop task from current goal set. Skipping planning"
            try:
                #print_state(pyhopState)
                # record attempt to replann
                self.mem.set(self.mem.PLANNING_COUNT, 1+self.mem.get(self.mem.PLANNING_COUNT))
                pyhopPlan = pyhop.pyhop(pyhopState, pyhopTasks, verbose = 0)
            except Exception:

                pyhopPlan = None
            if not pyhopPlan and pyhopPlan != []:
                if verbose >= 1:
                    print "Planning failed for ",
                    for goal in goals:
                        print goal, " ",
                    print

                if trace: trace.add_data("PLAN", None) # planning failed, record NONE for plan
                return
            #change from pyhop plan to MIDCA plan
            midcaPlan = plans.Plan([plans.Action(action[0], *list(action[1:])) for action in pyhopPlan], goals)

            if verbose >= 1:
                print "Planning complete."
            if verbose >= 2:
                print "Plan: "#, midcaPlan
                for a in midcaPlan:
                    print("  "+str(a))
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
            print "Error declaring pyhop methods and operators. This planner will be \
            disabled"
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
                print "No goals received by planner. Skipping planning."
            return
        try:
            midcaPlan = self.mem.get(self.mem.GOAL_GRAPH).getMatchingPlan(goals)
        except AttributeError:
            midcaPlan = None
        if midcaPlan:
            if verbose >= 2:
                print "Old plan retrieved. Checking validity...",
            valid = world.plan_correct(midcaPlan)
            if not valid:
                midcaPlan = None
                #if plan modification is added to MIDCA, do it here.
                if verbose >= 2:
                    print "invalid."
            elif verbose >= 2:
                print "valid."
            if valid:
                if verbose >= 2:
                    print "checking to see if all goals are achieved...",
                achieved = world.plan_goals_achieved(midcaPlan)
                if verbose >= 2:
                    if len(achieved) == len(midcaPlan.goals):
                        print "yes"
                    else:
                        print "no. Goals achieved: " + str({str(goal) for goal in achieved})
                if len(achieved) != len(midcaPlan.goals):
                    midcaPlan = None #triggers replanning.

        #ensure goals is a collection to simplify things later.
        if not isinstance(goals, collections.Iterable):
            goals = [goals]

        if not midcaPlan:
            #use pyhop to generate new plan
            if verbose >= 2:
                print "Planning..."
            try:
                pyhopState = self.pyhop_state_from_world(world)
            except Exception:
                print "Could not generate a valid pyhop state from current world state. Skipping planning"

            try:
                pyhopTasks = self.pyhop_tasks_from_goals(goals,pyhopState)
            except Exception:
                print "Could not generate a valid pyhop task from current goal set. Skipping planning"
            try:
                #print_state(pyhopState)
                # record attempt to replann
                self.mem.set(self.mem.PLANNING_COUNT, 1+self.mem.get(self.mem.PLANNING_COUNT))
                pyhopPlan = pyhop.pyhop(pyhopState, pyhopTasks, verbose = 0)
            except Exception:

                pyhopPlan = None
            if not pyhopPlan and pyhopPlan != []:
                if verbose >= 1:
                    print "Planning failed for ",
                    for goal in goals:
                        print goal, " ",
                    print

                if trace: trace.add_data("PLAN", None) # planning failed, record NONE for plan
                return
            #change from pyhop plan to MIDCA plan
            midcaPlan = plans.Plan([plans.Action(action[0], *list(action[1:])) for action in pyhopPlan], goals)

            if verbose >= 1:
                print "Planning complete."
            if verbose >= 2:
                print "Plan: "#, midcaPlan
                for a in midcaPlan:
                    print("  "+str(a))
            #save new plan
            if midcaPlan != None:
                self.mem.get(self.mem.GOAL_GRAPH).addPlan(midcaPlan)
            if trace: trace.add_data("PLAN",midcaPlan)


class HSPNode():
    '''
    A node that will be used in the Heuristic Search Planner
    '''
    world = None # A state in MIDCA
    parent_node = [] # an HSPNode (or None for root)
    actions_taken = [] # actions taken to reach this node (these are MIDCA actions)
    depth = 0

    def __init__(self, world, parent_node, actions_taken):
        self.world = world
        self.parent_node = parent_node
        if parent_node:
            self.depth = parent_node.depth+1
        else:
            self.depth = 0
        self.actions_taken = actions_taken

class HeuristicSearchPlanner(base.BaseModule):
    '''
    Heuristic Search Planner.

    When initialized, this planner needs to be provided with:
    1. Heuristic function that gives some value for a state (if none give, the depth will be the value)
    2. Decomposition function, that given a node, will return the child nodes (if none given, will be Breadth First Search)
    '''

    def __init__(self, hn=lambda n: n.get_depth(), dn=None):
        self.hn = hn

    def init(self, world, mem):
        self.world = world
        self.mem = mem
        self.mem.set(self.mem.PLANNING_COUNT, 0)

    def get_all_instantiations(self, world, operator):
        '''
        Returns all possible operator instantiations
        Note: If the state is more than 50 or so atoms,
              this could be a slow and expensive
              function
        '''

        # need to preserve order of elements of the following lists
        arg_names = []
        arg_types = []

        # initialize with objnames
        for o in operator.objnames:
            arg_names.append(o)
            arg_types.append(None)

        # find the correct types
        for precond in operator.preconditions:
            args = map(str,precond.atom.args)
            for i in range(len(args)):
                arg_names_i = arg_names.index(args[i])
                if arg_types[arg_names_i] is None:
                    arg_types[arg_names_i] = precond.argtypes[i]

        def get_type(t):
            return world.get_objects_by_type(t)

        # get all objects that match each type
        possible_bindings = map(get_type,arg_types)

        # generate all possible arrangements
        permutations = itertools.product(*possible_bindings)

        # now run through all possible bindings
        applicable_permutations = []
        num_permutations = 0
        for c in permutations:
            num_permutations+=1
            op_inst = operator.instantiate(list(c))
            op_inst.set_args(list(c))
            if world.is_applicable(op_inst):

                applicable_permutations.append(op_inst)
                # break # uncomment this line if you just want to get the first valid instantiation

        return applicable_permutations

    def get_instantiations_nbeacons(self, world, operator):
        '''
        Returns possible operator instantiations specifically optimized for operating in the NBeacons
        domain. This function returns a significantly smaller number of possible instantiations of an
        operator compared to instead of get_all_instantiations(). It is specific to the nbeacons domain.
        '''

        # using two lists instead of a dict to preserve order, could use an OrderedDict
        arg_names = []
        arg_types = []
        #print "operator.name = "+operator.name
        if 'move' in operator.name:
            # get agent's tile
            agent_loc = str(world.get_atoms(filters="agent-at")[0].args[1])
            #print "agent_loc "+str(agent_loc)
            #[0].args[1]
            # get all adjacent tiles within a radius of 3

            # radius = 1
            adjacent_atoms = world.get_atoms(filters=["adjacent",agent_loc])
            # radius = 2

            # LEFT OFF, need to make sure all atoms are available for operators
            # that require multiple adjacent
            # solution 1 - use moveeast2 or moveeast3 as operator names, and for those operators,
            #              do brute force expansion
            # solution 2 - brute force everything anyways because still should be fast enough
            #              TRIED THIS - WORKS but too slow
            # solution 3 - enlarge the radius of adjacent tiles and just use those.

            #for adj_atom in copy(adjacent_atoms):
            #    if adj_atom
            #print "adjacent atoms are " +str(adjacent_atoms)
            valid_tiles = []
            # this is hacky, but speeds up experiments
            #t_1 = time.time()
            if operator.name not in ['moveeast2','moveeast3','moveeast4', 'moveeast5']:
                for aa in adjacent_atoms:
                    #print "  aa.args[0] is " + str(aa.args[0])+" and "+str(aa.args[1])
                    valid_tiles.append(aa.args[0])
                    valid_tiles.append(aa.args[1])
            else:
                adjacent_atoms = world.get_atoms(filters=["adjacent-east",agent_loc])
                if operator.name == 'moveeast2':
                    for aa in adjacent_atoms:
                        #print "  aa.args[0] is " + str(aa.args[0])+" and "+str(aa.args[1])
                        valid_tiles.append(aa.args[0])
                        valid_tiles.append(aa.args[1])
                        double_adj_atoms = world.get_atoms(filters=["adjacent-east",str(aa.args[1])])
                        for da in double_adj_atoms:
                            valid_tiles.append(da.args[0])
                            valid_tiles.append(da.args[1])
                elif operator.name == 'moveeast3':
                    for aa in adjacent_atoms:
                        #print "  aa.args[0] is " + str(aa.args[0])+" and "+str(aa.args[1])
                        valid_tiles.append(aa.args[0])
                        valid_tiles.append(aa.args[1])
                        double_adj_atoms = world.get_atoms(filters=["adjacent-east",str(aa.args[1])])
                        for da in double_adj_atoms:
                            valid_tiles.append(da.args[0])
                            valid_tiles.append(da.args[1])
                            triple_adj_atoms = world.get_atoms(filters=["adjacent-east",str(da.args[1])])
                            for ta in triple_adj_atoms:
                                valid_tiles.append(ta.args[0])
                                valid_tiles.append(ta.args[1])
                elif operator.name == 'moveeast4':
                    for aa in adjacent_atoms:
                        #print "  aa.args[0] is " + str(aa.args[0])+" and "+str(aa.args[1])
                        valid_tiles.append(aa.args[0])
                        valid_tiles.append(aa.args[1])
                        double_adj_atoms = world.get_atoms(filters=["adjacent-east",str(aa.args[1])])
                        for da in double_adj_atoms:
                            valid_tiles.append(da.args[0])
                            valid_tiles.append(da.args[1])
                            triple_adj_atoms = world.get_atoms(filters=["adjacent-east",str(da.args[1])])
                            for ta in triple_adj_atoms:
                                valid_tiles.append(ta.args[0])
                                valid_tiles.append(ta.args[1])
                                quad_adj_atoms = world.get_atoms(filters=["adjacent-east",str(ta.args[1])])
                                for qa in quad_adj_atoms:
                                    valid_tiles.append(qa.args[0])
                                    valid_tiles.append(qa.args[1])
                elif operator.name == 'moveeast5':
                    for aa in adjacent_atoms:
                        #print "  aa.args[0] is " + str(aa.args[0])+" and "+str(aa.args[1])
                        valid_tiles.append(aa.args[0])
                        valid_tiles.append(aa.args[1])
                        double_adj_atoms = world.get_atoms(filters=["adjacent-east",str(aa.args[1])])
                        for da in double_adj_atoms:
                            valid_tiles.append(da.args[0])
                            valid_tiles.append(da.args[1])
                            triple_adj_atoms = world.get_atoms(filters=["adjacent-east",str(da.args[1])])
                            for ta in triple_adj_atoms:
                                valid_tiles.append(ta.args[0])
                                valid_tiles.append(ta.args[1])
                                quad_adj_atoms = world.get_atoms(filters=["adjacent-east",str(ta.args[1])])
                                for qa in quad_adj_atoms:
                                    valid_tiles.append(qa.args[0])
                                    valid_tiles.append(qa.args[1])
                                    cinco_adj_atoms = world.get_atoms(filters=["adjacent-east",str(qa.args[1])])
                                    for ca in cinco_adj_atoms:
                                        valid_tiles.append(ca.args[0])
                                        valid_tiles.append(ca.args[1])

            # uncomment next 5 lines to make quicksand visible
            quicksand_tiles = map(lambda atom: atom.args[0], world.get_atoms(filters=["quicksand"]))
            # if the agent's location is in quicksand, remove it
            quicksand_tiles = [t for t in quicksand_tiles if str(t) != agent_loc]
            valid_tiles = set(valid_tiles) - set(quicksand_tiles)
            for vt in valid_tiles:
                if str(vt) in quicksand_tiles:
                    raise Exception("Failing to remove quicksand tile"+str(vt)+" from search")

            #if operator.name in ['moveeast2','moveeast3','moveeast4', 'moveeast5']:
            #    t_2 = time.time()
            #    t_str =  '%.2f' % (t_2-t_1)
                #print "getting valid tiles for "+str(operator.name)+" took "+str(t_str)+"s"
            #    print "valid tiles are "+str(map(str,valid_tiles))

            #print "quicksand tiles are "+str(map(str,quicksand_tiles))
            #valid_tiles = filter(lambda vt: not (str(vt) in quicksand_tiles), valid_tiles)

            #print "valid tiles are "+str(map(str,valid_tiles))

            # this is a hack for now

            # possible tiles
            #for vt in valid_tiles:
            #    print "  valid tile "+str(vt)

        if 'beacon' in operator.name:
            beacon_atoms = world.get_atoms(filters=["beacon-at"])
            beacon_tiles = map(lambda b: b.args[1],beacon_atoms)
            valid_tiles = beacon_tiles

        for o in operator.objnames:
            arg_names.append(o)
            arg_types.append(None)

        #print "arg_names_and_types = "+str(zip(arg_names,arg_types))
        for precond in operator.preconditions:
            #print "precond is "+str(precond)
            #print "precond atom is "+str(precond.atom)
            args = map(str,precond.atom.args)
            #print "args are "+str(map(str,args))
            for i in range(len(args)):
                arg_names_i = arg_names.index(args[i])
                if arg_types[arg_names_i] is None:
                    arg_types[arg_names_i] = precond.argtypes[i]

        #print "arg_names_and_types = "+str(zip(arg_names,map(str,arg_types)))

        def better_mapping_func(t):
            #print "t is "+str(t)
            if 'TILE' in str(t):
                #print "we are here, returning "+str(valid_tiles)
                return valid_tiles
            else:
                return world.get_objects_by_type(t)

        def old_mapping_func(t):
            return world.get_objects_by_type(t)

        possible_bindings = map(better_mapping_func,arg_types)
        #possible_bindings = map(old_mapping_func,arg_types)
        #print "here"
#         if operator.name in ['moveeast2','moveeast3','moveeast4']:
#             for pb in possible_bindings:
#                 print "outer"
#                 for pb_i in pb:
#                     print "  "+str(pb_i)
        permutations = itertools.product(*possible_bindings)
#         i_s = 0
#         i_e = 5
#         print "there are "+str(len(permutations))+" permutations"
#         for p in permutations:
#             print "p = "+str(map(str,p))
#             time.sleep(1)
#
        applicable_permutations = []
        # need to do a transpose on the permutations
        #permutations = zip(*permutations)

        num_permutations = 0
        for c in permutations:
            num_permutations+=1
            #print "len(c) = "+str(len(c))
            #print "c = "+str(map(str,c))
            #print "attempting to instantiate operator "+str(operator)#+" with args "+str(map(str,c))
            op_inst = operator.instantiate(list(c))
            #print "successfully instantiated the following operator:"
            #print operator
            op_inst.set_args(list(c))
            if world.is_applicable(op_inst):
                #print "just instantiated operator "+str(operator)+" with args "+str(map(str,c))
                applicable_permutations.append(op_inst)
                #if operator.name in ['moveeast2','moveeast3','moveeast4', 'moveeast5']:
                    #print "took "+str(num_permutations)+" permutations before "+str(operator.name)+" worked "
                break
        #print "here2"
        #print " there are at least "+str(num_permutations)+" for operator "+str(operator.name)

        #time.sleep(5)
        #print "preobjnames are: " +str(operator.preobjnames)
        #print "preobjtypes are: " +str(operator.preobjtypes)
        #for perm in applicable_permutations:
        #    print "perm = "+str(perm)

        return applicable_permutations


    def brute_force_decompose(self, node, visited):
        '''
        get all operators (before finding variable bindings) in MIDCA
        '''

        child_nodes = []

        available_operators = node.world.operators.values()

        for op in available_operators:
            inst_operators = self.get_all_instantiations(node.world,op)
            for inst_op in inst_operators:

                new_world = node.world.copy()

                try:
                    new_world.apply(inst_op)
                except:
                    print("====== Tried to apply action:")
                    print str(inst_op)
                    print("====== On world:")
                    print str(new_world)
                    print("====== But Failed:")

                already_visited = False

                for w in map(lambda n: n.world, visited):
                    if new_world.equal(w):
                        already_visited = True

                if not already_visited:
                    child =  HSPNode(new_world.copy(),node,node.actions_taken+[inst_op])
                    child_nodes.append(child)

        return child_nodes

    def brute_force_decompose_nbeacons(self, node, visited):
        '''
        Get all operators (before finding variable bindings) in MIDCA

        This function is specific to NBeacons domain with MIDCA.
        See brute_force_decompose() for a general solution
        '''

        child_nodes = []

        available_operators = node.world.operators.values()

        is_stuck = len(node.world.get_atoms(filters=['stuck'])) > 0
        if is_stuck:
            available_operators = filter(lambda op: 'push' in str(op), node.world.operators.values())

        for op in available_operators:
            inst_operators = self.get_instantiations_nbeacons(node.world,op)

            for inst_op in inst_operators:

                #node.world.apply_midca_action(inst_op)
                #print str(inst_op)+'\n ==== is of type '+ str(type(inst_op))+" dir(action) = \n"+str(dir(inst_op))
                #print " looking at operator "+str(inst_op.operator.name)
                #return
                new_world = node.world.copy()

                try:
                    new_world.apply(inst_op)
                except:
                    print("====== Tried to apply action:")
                    print str(inst_op)
                    print("====== On world:")
                    print str(new_world)
                    print("====== But Failed:")
                # check to make sure its not
                already_visited = False
                #print "visited = "+str(visited)
                #print "diffs with current world = "
                #for i in map(lambda n: new_world.diff(n.world),visited):
                #    print "  ("+str(map(str,i[0]))+","+str(map(str,i[1]))+")"
                for w in map(lambda n: n.world, visited):
                    if new_world.equal(w):
                        already_visited = True
                #print "Already Visited is "+str(already_visited)
                # add
                if not already_visited:
                    child =  HSPNode(new_world.copy(),node,node.actions_taken+[inst_op])
                    child_nodes.append(child)
                    #print "adding child node with operator "+str(inst_op.operator.name)+" and depth "+str(child.depth)

        return child_nodes


    def nbeacons_heuristic(self,goals,infinity=10000):
        # first define internal heuristic, then return it

        def old_heuristic(node):
            DEPTH_MULTIPLIER = 0.8
            # if 'free' is in goals, than rank push nodes higher
            goal_type_free_goal = False
            for goal in goals:
                if 'free' in str(goal):
                    goal_type_free_goal = True

            # after checking free, if agent-at is in goals, rank move nodes higher
            goal_type_agent_at = False
            for goal in goals:
                if 'agent-at' in str(goal):
                    goal_type_agent_at = True

            # check to see if this is a beacon activation node
            goal_type_beacon_activation = False
            for goal in goals:
                if 'activated' in str(goal):
                    goal_type_beacon_activation = True

            if goal_type_free_goal:
                # all push actions
                exists_push_action = False
                all_push_actions = True
                for action in node.actions_taken:
                    if 'push' in action.operator.name:
                        exists_push_action = True
                    else:
                        all_push_actions = False

                if exists_push_action and all_push_actions:
                    return node.depth # depth only
                else:
                    return infinity+node.depth

            # we will use manhatten distance to sort nodes for both
            # beacon activation and navigation type goals
            elif goal_type_agent_at:
                # check that all actions are move actions
                exists_move_action = False
                all_move_actions = True
                for action in node.actions_taken:
                    if 'move' in action.operator.name:
                        exists_move_action = True
                    else:
                        all_move_actions = False

                if exists_move_action and all_move_actions:
                    # now compute distance because its relevant
                    agent_loc = node.world.get_atoms(filters=['agent-at'])[0]
                    agent_loc = str(agent_loc.args[1])[2:] # remove the 'Tx'
                    agent_x = int(agent_loc.split('y')[0])
                    agent_y = int(agent_loc.split('y')[1])
                    goal_loc = str(goals[0].args[1])[2:]
                    goal_x = int(goal_loc.split('y')[0])
                    goal_y = int(goal_loc.split('y')[1])

                    dist = (abs(goal_x - agent_x)+abs(goal_y-agent_y))
                    return dist+(DEPTH_MULTIPLIER*node.depth)
                else:
                    # these nodes have actions other than movement, not relevant
                    return infinity+node.depth

            elif goal_type_beacon_activation:
                # get current location of agent
                #print "goals are "+str(goals)
                #print "goal is "+str(goals[0])
                #print "activated goal dir "+str(map(str,goals[0].args))

                #print "agent_loc = "+str(agent_loc.args[1])
                #print "activated goal args "+str(map(str,goals[0].args))

                #print "beacon loc = "+str(beacon_atom.args[1])
                # get location of beacon

                # check that all actions are move actions, allow for one activate beacon action
                exists_non_move_or_beacon_action = False
                num_beacon_activate_actions = 0
                for action in node.actions_taken:
                    if 'beacon' in action.operator.name:
                        num_beacon_activate_actions+=1
                    elif not ('move' in action.operator.name):
                        exists_non_move_or_beacon_action = True

                if num_beacon_activate_actions <= 1 and not exists_non_move_or_beacon_action:
                    # now distance is relevant
                    agent_loc = node.world.get_atoms(filters=['agent-at'])[0]
                    agent_loc = str(agent_loc.args[1])[2:] # remove the 'Tx'
                    agent_x = int(agent_loc.split('y')[0])
                    agent_y = int(agent_loc.split('y')[1])

                    beacon_atom = node.world.get_atoms(filters=['beacon-at',goals[0].args[0]])[0]
                    beacon_loc = str(beacon_atom.args[1])[2:] # remove the 'Tx' at the front
                    goal_x = int(beacon_loc.split('y')[0])
                    goal_y = int(beacon_loc.split('y')[1])

                #print "agent_x = "+str(agent_x)+" agent y = "+str(agent_y) + " goal x = "+str(goal_x)+" goal y "+str(goal_y)

                #(abs(goal_x - node.agent_loc[0])

                    dist = abs(goal_x - agent_x) + abs(goal_y - agent_y)
                #print "dist is "+str(dist)
                    return dist+(DEPTH_MULTIPLIER*node.depth)
                else:
                    return infinity
            else:
                return infinity # this shouldn't happen because it means we have a different goal
            # END HEURISTIC FUNCTION

        def push_heuristic(node):
            # all push actions
            exists_push_action = False
            all_push_actions = True
            for action in node.actions_taken:
                if 'push' in action.operator.name:
                    exists_push_action = True
                else:
                    all_push_actions = False

            if exists_push_action and all_push_actions:
                return node.depth # depth only
            else:
                return infinity+node.depth


        def new_heuristic(node):
            DEPTH_MULTIPLIER = 0.8
            #is_stuck = len(node.world.get_atoms(filters=['stuck'])) > 0
            #if is_stuck:
            #    if 'push' not in map(str,node.actions_taken):
            #        return infinity
            # if the goal is an agent-at, or beacon, get goal_x and goal_y functions
            goal_x = -1
            goal_y = -1
            if 'agent-at' in str(goals[0]):
                goal_loc = str(goals[0].args[1])[2:]
                goal_x = int(goal_loc.split('y')[0])
                goal_y = int(goal_loc.split('y')[1])
            elif 'activated' in str(goals[0]):
                relevant_beacon_atoms = node.world.get_atoms(filters=['beacon-at',goals[0].args[0]])
                if len(relevant_beacon_atoms) == 0:
                    raise Exception("Goal is "+str(goals[0])+" and no relevant beacon atoms for goals[0].args[0] = "+str(goals[0].args[0]))
                beacon_atom = relevant_beacon_atoms[0]
                beacon_loc = str(beacon_atom.args[1])[2:] # remove the 'Tx' at the front
                goal_x = int(beacon_loc.split('y')[0])
                goal_y = int(beacon_loc.split('y')[1])

            # now compute distance because its relevant
            agent_loc = node.world.get_atoms(filters=['agent-at'])[0]
            agent_loc = str(agent_loc.args[1])[2:] # remove the 'Tx'
            agent_x = int(agent_loc.split('y')[0])
            agent_y = int(agent_loc.split('y')[1])

            dist = (abs(goal_x - agent_x)+abs(goal_y-agent_y))
            return dist+(DEPTH_MULTIPLIER*node.depth)

        # return a different heuristic based on the goal
        for goal in goals:
            if 'free' in str(goal):
                return push_heuristic

        return new_heuristic # now return the internal function

    def heuristic_search(self, goals, decompose):
        INFINITY = 10000
        #print "decompose is "+str(decompose)
        t0 = time.time()
        if not decompose:
            #print "****************using built-in***************** "
            decompose = self.brute_force_decompose

        Q = [HSPNode(self.world, None, [])]
        visited = []
        goal_reached_node = None
        nodes_expanded = 0
        while len(Q) != 0:
            # print Q
            #print "---- Q ----"

            #i = 0
            #for n in Q:
            #    print "  Node "+str(i)+": h(n)="+str(self.nbeacons_heuristic(goals)(n))+", actions="+str(map(lambda a:a.operator.name,n.actions_taken))+", depth = "+str(n.depth)
            #    i+=1

            # take the first node off the queue
            curr_node = Q[0]
            if self.verbose >=2:# or we_learned_an_op:
                print "-- len(Q): "+str(len(Q))+", "+str(nodes_expanded)+" n, a = "+str(map(lambda a:a.operator.name,curr_node.actions_taken)) + " h(n) = "+str(self.nbeacons_heuristic(goals)(curr_node))

            #print "expanding node "+str(id(curr_node))+" with depth "+str(curr_node.depth)
            #print "Expanding node with plan "+str(map(lambda a: str(a.operator.name),curr_node.actions_taken))+" and depth "+str(curr_node.depth)
            Q = Q[1:]
            visited.append(curr_node)
            nodes_expanded+=1
            # test if goal is reached
            if curr_node.world.goals_achieved_now(goals):
                goal_reached_node = curr_node
                break

            # if not, get child nodes
            Q += decompose(curr_node, visited)
            Q = sorted(Q,key=self.nbeacons_heuristic(goals,infinity=INFINITY))
            # now remove any node has a score >= infinity (because it's not relevant
            Q = filter(lambda s: self.nbeacons_heuristic(goals,infinity=INFINITY)(s) < INFINITY, Q)
            # also remove any node that has an activate beacon action that is not the last action
            def bad_activate(n):
                try:
                    activate_index = map(lambda a:a.operator.name,n.actions_taken).index('activatebeacon')
                    last_element_index = len(n.actions_taken) - 1
                    return activate_index == last_element_index
                except ValueError:
                    return True

            Q = filter(lambda n: bad_activate(n), Q)

        if goal_reached_node:
            t1 = time.time()
            timestr = '%.5f' % (t1-t0)
            if self.verbose >= 1: print "Heuristic Search Planning took "+timestr+"s"
            return goal_reached_node.actions_taken
        else:
            if self.verbose >= 1: print "Heuristic Search failed to produce a plan"
            return []

    def run(self, cycle, verbose = 2):
        self.verbose = verbose

	try:
            goals = self.mem.get(self.mem.CURRENT_GOALS)[-1]
        except:
            goals = []

        midcaPlan = None

        if not goals:
            if verbose >= 2:
                print "No goals received by planner. Skipping planning."
            return
        try:
            midcaPlan = self.mem.get(self.mem.GOAL_GRAPH).getMatchingPlan(goals)

            # check to see that midcaPlan has not been finished
            if midcaPlan.finished():
                # remove from goals and trigger replanning
                self.mem.get(self.mem.GOALGRAPH).removePlan(midcaPlan)
                if self.verbose >= 1: print "Old plan finished, will re-plan"
                midcaPlan = None
        except AttributeError:
            midcaPlan = None
            if verbose >= 2:
                print "Did not retrieve plan, will plan from scratch"

        # ensure goals is a collection to simplify things later.
        if not isinstance(goals, collections.Iterable):
            goals = [goals]

        if midcaPlan:
            if self.verbose >= 1: print "Retrieved current plan. Skipping planning."
            return

        if not midcaPlan:
            #use pyhop to generate new plan
            if verbose >= 2:
                print "Planning..."
            #try:
            self.mem.set(self.mem.PLANNING_COUNT, 1+self.mem.get(self.mem.PLANNING_COUNT))
            #print "Goals are "+str(map(str,goals))

            hsp_plan = self.heuristic_search(goals, decompose=self.brute_force_decompose_nbeacons)
            if self.verbose >= 1:
                print "planning finished: "
                for p in hsp_plan:
                    print "  "+str(p.operator.name)+"("+str(map(lambda o:o.name,p.args))+")"

            midcaPlan = plans.Plan([plans.Action(action.operator.name, *map(lambda o:o.name,action.args)) for action in hsp_plan], goals)

            if verbose >= 2:
                print "Plan: "#, midcaPlan
                for a in midcaPlan:
                    print("  "+str(a))

            if midcaPlan != None:
                self.mem.get(self.mem.GOAL_GRAPH).addPlan(midcaPlan)

