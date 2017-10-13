from _plan import pyhop
from midca.domains.blocksworld.plan import methods_broken, operators, operators_extinguish, methods_midca, operators_midca
from midca import plans, base
import collections
import copy
import traceback

class PyHopPlannerBroken(base.BaseModule):

    '''
    MIDCA module that implements a python version of the SHOP hierarchical task network (HTN) planner. HTN planners require a set of user-defined methods to generate plans; these are defined in the methods python module and declared in the constructor for this class.
    Note that this module uses has several methods to translate between MIDCA's world and goal representations and those used by pyhop; these should be changed if a new domain is introduced.
    '''

    pyhop_state_from_world = None
    pyhop_tasks_from_goals = None
    init_args = []
    
    def __init__(self,
                 pyhop_state_from_world,
                 pyhop_tasks_from_goals,
                 declare_methods,
                 declare_operators,
                 extinguishers = False,
                 mortar = False):
        self.init_args = [pyhop_state_from_world,
                          pyhop_tasks_from_goals,
                          declare_methods,
                          declare_operators,
                          extinguishers,
                          mortar]
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
        
    def get_init_args(self):
        '''
        init_args store the args that construct this object, so that
        when the metareasoner replaces this module with a working one,
        it can pass the same arguments to the new one. This function
        essentially preserves knowledge related to this module (which in this
        case is the planning module).
        
        TODO: build a wrapper around modules that store there init args behind
        the scenes, and when swapping modules, give access to the old args
        '''
        return self.init_args
    
    def init(self, world, mem):
        self.world = world
        self.mem = mem
        self.mem.set(self.mem.PLANNING_COUNT, 0)
    
    #this will require a lot more error handling, but ignoring now for debugging.
    def run(self, cycle, verbose = 2):
        #world = self.mem.get(self.mem.STATES)[-1]
        world = self.mem.get(self.mem.STATE)
        goals = self.mem.get(self.mem.CURRENT_GOALS)

        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("WORLD", copy.deepcopy(world))
            trace.add_data("GOALS", copy.deepcopy(goals))
            trace.add_data("PLAN", None)

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
                pyhopTasks = self.pyhop_tasks_from_goals(goals, pyhopState)
            except Exception as e:
                print e
                print "Could not generate a valid pyhop task from current goal set. Skipping planning"
            try:
                pyhopPlan = pyhop.pyhop(pyhopState, pyhopTasks, verbose = 0)
            except Exception:
                pyhopPlan = None
            if not pyhopPlan and pyhopPlan != []:
                if verbose >= 1:
                    print "Planning failed for ",
                    for goal in goals:
                        print goal, " ",
                    print
                if trace: trace.add_data("PLAN", pyhopPlan)
                return
            #change from pyhop plan to MIDCA plan
            midcaPlan = plans.Plan([plans.Action(action[0], *list(action[1:])) for action in pyhopPlan], goals)

            if verbose >= 1:
                print "Planning complete."
            if verbose >= 2:
                print "Plan: ", midcaPlan
            #save new plan
            if midcaPlan != None:
                self.mem.get(self.mem.GOAL_GRAPH).addPlan(midcaPlan)
            if trace: trace.add_data("PLAN",midcaPlan)
