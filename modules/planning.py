from _plan import pyhop
from MIDCA import plans, base
import collections
import traceback
import copy
from MIDCA.modules._plan.asynch import asynch
from MIDCA.modules._plan.pyhop import print_state,  print_methods, print_operators
import time

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

    def __init__(self, declare_methods, declare_operators, plan_validator = None):
        try:
            declare_methods()
            declare_operators()
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
        goals = self.mem.get(self.mem.CURRENT_GOALS)
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

    def __init__(self, declare_methods, declare_operators):
                GenericPyhopPlanner.__init__(self, declare_methods,declare_operators,
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

        goals = self.mem.get(self.mem.CURRENT_GOALS)
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
            plan = self.get_new_plan(state, goals, verbose)
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
        
    #this will require a lot more error handling, but ignoring now for debugging.
    def run(self, cycle, verbose = 2):
        world = self.mem.get(self.mem.STATES)[-1]
        goals = self.mem.get(self.mem.CURRENT_GOALS)

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
                if trace: trace.add_data("PLAN", pyhopPlan)
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

class PyHopPlanner2(base.BaseModule):

    '''
    OLD #TODO
    MIDCA module that implements a python version of the SHOP hierarchical task network (HTN) planner. HTN planners require a set of user-defined methods to generate plans; these are defined in the methods python module and declared in the constructor for this class.
    Note that this module uses has several methods to translate between MIDCA's world and goal representations and those used by pyhop; these should be changed if a new domain is introduced.
    '''

    def __init__(self, extinguishers = False):
        #declares pyhop methods. This is where the planner should be given the domain information it needs.
        if extinguishers:
            methods_extinguish.declare_methods()
            operators_extinguish.declare_ops()
        else:
            methods_midca.declare_methods()
            operators_midca.declare_ops()

    #this will require a lot more error handling, but ignoring now for debugging.
    def run(self, cycle, verbose = 2):
        world = self.mem.get(self.mem.STATES)[-1]
        goals = self.mem.get(self.mem.CURRENT_GOALS)
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
                pyhopState = world
            except Exception:
                print "Could not generate a valid pyhop state from current world state. Skipping planning"
            try:
                pyhopTasks = pyhop_tasks_from_goals(goals)
            except Exception:
                print "Could not generate a valid pyhop task from current goal set. Skipping planning"
            #try:
            pyhopPlan = pyhop.pyhop(pyhopState, pyhopTasks, verbose = 0)
            #except Exception:
                #pyhopPlan = None
            if not pyhopPlan and pyhopPlan != []:
                if verbose >= 1:
                    print "Planning failed for ",
                    for goal in goals:
                        print goal, " ",
                    print
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


class HSPNode():
    '''
    A node that will be used in Heuristic Search Planner
    '''
    state = None # A state in MIDCA
    parent_node = []
    actions_taken = [] # actions taken to reach this node
    depth = 0
    
    def __init__(self, state, parent_node, actions_taken):
        self.state = state
        self.parent_node = parent_node
        if parent_node:
            self.depth = parent_node.depth+1
        else:
            self.depth = 0
        self.actions_taken = actions_taken

    def __str__(self):
        s = "state_x_len="+str(len(self.state))+"state_y_len"+str(len(self.state[0]))
        return s

import itertools

class HeuristicSearchPlanner(base.BaseModule):
    '''
    Heuristic Search Planner.
    
    When initialized, this planner needs to be provided with:
    1. Heuristic function that gives some value for a state (if none give, the depth will be the value)
    2. Decomposition function, that given a node, will return the child nodes (if none given, will be Breadth First Search)
    '''
    
    def __init__(self, hn=lambda n: n.get_depth(), dn=None):
        self.hn = hn
        #if not dn:
        #    self.dn = self.bfs_dn
    
    def init(self, world, mem):
        self.world = world
        self.mem = mem
    
    def get_all_instantiations(self, operator):
        '''
        Returns all possible operator instantiations
        '''
        
        
        # go through each precondition
        #print "operator is "+str(operator)
        #print "operator takes args: " +str(operator.objnames)
        
        possible_arg_values = {}
        # using two lists instead of a dict to preserve order
        arg_names = []
        arg_types = []
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
        
        possible_bindings = map(lambda t: self.world.get_objects_by_type(t),arg_types)
        #for pb in possible_bindings:
        #    print "outer"
        #    for pb_i in pb:
        #        print "  "+str(pb_i)
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
        for c in permutations:
            #print "len(c) = "+str(len(c))
            #print "c = "+str(c)
            #print "attempting to instantiate operator "+str(operator)+" with args "+str(map(str,c))
            if self.world.is_applicable(operator.instantiate(list(c))):
                #print "just instantiated operator "+str(operator)+" with args "+str(map(str,c))
                applicable_permutations.append(operator.instantiate(list(c)))

        
        
        #time.sleep(5)            
        #print "preobjnames are: " +str(operator.preobjnames)
        #print "preobjtypes are: " +str(operator.preobjtypes)
        #for perm in applicable_permutations:
        #    print "perm = "+str(perm)
        
        return applicable_permutations
        
        
        #print "types = "
        #for t in self.world.types:
        #    print "  t = "+t
        #print "all objects of each type"
        
        #print map(str,map(lambda o: o.type,self.world.get_possible_objects('_','_')))
        for pre in operator.preconditions:
            print "argtypes are "
            print "  " +str(map(str,pre.argtypes))
            possible_bindings = map(lambda t: self.world.get_objects_by_type(t),pre.argtypes)
            print "possible bindings are"
            pb = map(str,possible_bindings)
            for pb_i in range(len(possible_bindings)):
                print "There are "+str(len(possible_bindings[pb_i]))+ "following are possible bindings for "+str(pre.argtypes[pb_i])
                #print possible_bindings[pb_i]
                #for pb_ii in possible_bindings[pb_i]:
                #    print "  "+str(pb_ii)
#             pred_name = pre.atom.predicate.name
#             # go through each arg type
#             for arg in pre.atom.args
#             
        self.world.get_objects_by_type('TILE')
        return
#         possible_arg_values[pre] = map(str,self.world.get_atoms(filters=[pred_name])) 
    
        # now list all possible permutations
        #print "input to product is "+str(possible_arg_values.values())
        print " values per arg = "+str({k:len(v) for k,v in possible_arg_values.items()})
        permutations = itertools.product(*possible_arg_values.values())
        size_permutations = sum(1 for _ in permutations) # space efficient way of computing permutations
        print " there are "+str(size_permutations)+" permuatations"
        
        # now go through all possible args, and record how many are applicable
        valid_instantiations = []
        for p in permutations:
            if self.world.is_applicable(operator.instantiate(*p)):
                print ("found valid instantiation: "+str(p))
                valid_instantiations = [p]
        
        t1 = time.time()
        timestr = '%.5f' % (t1-t0)
        print("Took "+timestr+"s to get all "+str(len(valid_instantiations))+" instantiations of "+str(operator.name))
        
    def brute_force_decompose(self, node):
        # get all operators (pre-variable bindings) in MIDCA
        #print str(self.world.operators)
        t0 = time.time()
        possible_operators = []
        for op in self.world.operators.values():
            print "OPERATOR "+op.name
            inst_operators = self.get_all_instantiations(op)
            possible_operators += inst_operators
            print "Instantiation: "
            print str(map(str,inst_operators))
            
        
        t1 = time.time()
        timestr = '%.5f' % (t1-t0)
        print("Took "+timestr+"s to get all "+str(len(possible_operators))+" instantiations of operators")
#             
#             print "Looking at "+str(op.name)
#             for pre in op.preconditions:
#                 print "  pre is "+str(pre.atom.predicate.name)
#                 pred_name = pre.atom.predicate.name
#                 # get all atoms that match predicate name
#                 for atm in self.world.get_atoms(filters=[pred_name]):
#                     print "    "+str(atm)
#             
            
            
    
    
#     def decompose(self, node):
#         '''
#         Returns the possible child nodes
#         '''
#     
#         # Go through all operators (pre-variable bindings) in MIDCA
#         for raw_op in operators:
#             # go through all predicates in the op's preconditions
#             for pred in raw_op.get_preconditions:
#                 # retrieve all 
#     
    def run(self, cycle, verbose = 2):
        self.brute_force_decompose(None)