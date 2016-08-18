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
    world = None # A state in MIDCA
    parent_node = []
    actions_taken = [] # actions taken to reach this node
    depth = 0
    
    def __init__(self, world, parent_node, actions_taken):
        self.world = world
        self.parent_node = parent_node
        if parent_node:
            self.depth = parent_node.depth+1
        else:
            self.depth = 0
        self.actions_taken = actions_taken

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
    
    def get_all_instantiations(self, world, operator):
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
        if 'move' in operator.name:
            # get agent's tile
            agent_loc = str(world.get_atoms(filters="agent-at")[0].args[1])
            #print "agent_loc "+str(agent_loc)
            #[0].args[1]
            # get all adjacent tiles
            adjacent_atoms = world.get_atoms(filters=["adjacent",agent_loc])
            print "adjacent atoms are " +str(adjacent_atoms)
            valid_tiles = []
            for aa in adjacent_atoms:
                print "  aa.args[0] is " + str(aa.args[0])+" and "+str(aa.args[1])
                valid_tiles.append(aa.args[0])
                valid_tiles.append(aa.args[1])
            valid_tiles = set(valid_tiles)
            # possible tiles
            #for vt in valid_tiles:
            #    print "  valid tile "+str(vt)
            
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
        
        print "arg_names_and_types = "+str(zip(arg_names,map(str,arg_types)))
        
        def better_mapping_func(t):
            print "t is "+str(t)
            if 'TILE' in str(t):
                return valid_tiles
            else:
                return world.get_objects_by_type(t)
        
        def old_mapping_func(t):
            return world.get_objects_by_type(t)
        
        possible_bindings = map(better_mapping_func,arg_types)
        print "here"
        for pb in possible_bindings:
            print "outer"
            for pb_i in pb:
                print "  "+str(pb_i)
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
            #print "c = "+str(c)
            #print "attempting to instantiate operator "+str(operator)+" with args "+str(map(str,c))
            op_inst = operator.instantiate(list(c))
            op_inst.set_args(list(c))
            if world.is_applicable(op_inst):
                #print "just instantiated operator "+str(operator)+" with args "+str(map(str,c))
                applicable_permutations.append(op_inst)
                break
        print " there are at least "+str(num_permutations)+" for operator "+str(operator.name)
        
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
        
    def brute_force_decompose(self, node, visited):
        # get all operators (pre-variable bindings) in MIDCA
        #print str(self.world.operators)
        t0 = time.time()
        child_nodes = []
        t1=t0
        for op in node.world.operators.values():
            inst_operators = self.get_all_instantiations(node.world,op)
            t2 = time.time()
            timestr = '%.5f' % (t2-t1)
            #print("Took "+timestr+"s to get "+str(len(inst_operators))+ " instantiation(s) of "+str(map(lambda a: a.operator.name,inst_operators)))
            t1=t2
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
            
        t1 = time.time()
        timestr = '%.5f' % (t1-t0)
        #print("Took "+timestr+"s to get all "+str(len(child_nodes))+" child nodes")
        return child_nodes
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

    def nbeacons_heuristic(self,goals):
        
        def heuristic(node):
            # if 'free' is in goals, than rank push nodes higher
            has_free_goal = False
            for goal in goals:
                if 'free' in str(goal):
                    has_free_goal = True
            
            # after checking free, if agent-at is in goals, rank move nodes higher
            agent_at_goal = False
            for goal in goals:
                if 'agent-at' in str(goal):
                    agent_at_goal = True
            
            if has_free_goal:
                # all push actions
                exists_push_action = False
                all_push_actions = True
                for action in node.actions_taken:
                    if 'push' in action.operator.name:
                        exists_push_action = True
                    else:
                        all_push_actions = False
                        
                if exists_push_action and all_push_actions:
                    return 0+node.depth
            
            elif agent_at_goal:
                # all move actions
                exists_move_action = False
                all_move_actions = True
                for action in node.actions_taken:
                    if 'move' in action.operator.name:
                        exists_move_action = True
                    else:
                        all_move_actions = False
                        
                if exists_move_action and all_move_actions:
                    return 0+node.depth
            
            return 100+node.depth
        return heuristic

    def heuristic_search(self, goals, decompose):
        #print "decompose is "+str(decompose)
        t0 = time.time()
        if not decompose:
            #print "****************using built-in***************** "
            decompose = self.brute_force_decompose
            
        Q = [HSPNode(self.world, None, [])]
        visited = []
        goal_reached_node = None
        while len(Q) != 0:
            # print Q
            #print "  -- Q --  "
            #i = 0
            #for n in Q:
            #    print "Node "+str(i)+": h(n)="+str(self.nbeacons_heuristic(goals)(n))+", actions="+str(map(lambda a:a.operator.name,n.actions_taken))+", depth = "+str(n.depth)
            #    i+=1
            
            # take the first node off the queue
            curr_node = Q[0]
            #print "expanding node "+str(id(curr_node))+" with depth "+str(curr_node.depth)
            #print "Expanding node with plan "+str(map(lambda a: str(a.operator.name),curr_node.actions_taken))+" and depth "+str(curr_node.depth)
            Q = Q[1:]
            visited.append(curr_node)
            
            # test if goal is reached
            if curr_node.world.goals_achieved_now(goals):
                goal_reached_node = curr_node
                break
            
            # if not, get child nodes
            Q += decompose(curr_node, visited)
            Q = sorted(Q,key=self.nbeacons_heuristic(goals))    
        
        if goal_reached_node:
            t1 = time.time()
            timestr = '%.5f' % (t1-t0)
            print "Heuristic Search Planning took "+timestr+"s"
            return goal_reached_node.actions_taken
        else:
            print "Heuristic Search failed to produce a plan"
            return []
        
    def run(self, cycle, verbose = 2):
        world = self.mem.get(self.mem.STATES)[-1]
        goals = self.mem.get(self.mem.CURRENT_GOALS)
        
        midcaPlan = None
        
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
                self.mem.set(self.mem.PLANNING_COUNT, 1+self.mem.get(self.mem.PLANNING_COUNT))
                #print "Goals are "+str(map(str,goals))
                hsp_plan = self.heuristic_search(goals, decompose=None)
                #print "planning finished: "
                for p in hsp_plan:
                    print "  "+str(p.operator.name)+"("+str(map(lambda o:o.name,p.args))+")"
                
                midcaPlan = plans.Plan([plans.Action(action.operator.name, *map(lambda o:o.name,action.args)) for action in hsp_plan], goals)
                
                if verbose >= 2:
                    print "Plan: "#, midcaPlan
                for a in midcaPlan:
                    print("  "+str(a))
                    
                if midcaPlan != None:
                        self.mem.get(self.mem.GOAL_GRAPH).addPlan(midcaPlan)
            except:
                print "Planning Failed, skipping"