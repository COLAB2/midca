import os,copy
import importlib
from midca import base, goals, modules
from midca.modules.gens import goaltransform


class MRSimpleControl(base.BaseModule):
    midca = None
    prev_init_args = [] # TODO: fix, right now used only for planning and saving args
                        # from a remove module action and then using the args to 
                        # insert a new module action
    
    def run(self, cycle, verbose = 2):
        self.verbose = verbose
        plan = self.mem.get(self.mem.META_PLAN)
        self.prev_init_args = []
        failed = False
        if plan:
            for action in plan:
                if not self.act(action):
                    failed = True
                    if self.verbose >= 1: print("    MRSimpleControl failed to complete an action")
                    self.cleanup_and_reset()
            if not failed:
                self.cleanup_and_reset()
        else:
            if self.verbose >= 1: print("    No actions taken")



    def cleanup_and_reset(self):
        """
        Temporary function to remove goal and plan,
        should eventually be replaced by evaluate
         """
        self.mem.set(self.mem.META_PLAN, None)
        self.mem.set(self.mem.META_GOALS, None)
        self.mem.set(self.mem.META_CURR_GOAL, None)
        self.mem.set(self.mem.PLAN, [])

    def act(self, action, verbose = 2):
        # TODO: figure out a way to make the init_args more general (so actions can be kept separate)
        if action[0] == "REMOVE-MODULE":
            # find the component
            module_index = -1
            phase = None
            mod_str = ""
            class Found(Exception): pass # is this bad python? should this go at top of my file?
            try:
                for phasei in self.mem.myMidca.get_phases():
                    i = 0
                    for mod in self.mem.myMidca.get_modules(phasei):
                        mod_str = str(mod.__class__.__name__)
                        print("-*-*- act():  mod = "+mod_str+", action[1] = "+str(action[1]))
                        if mod_str == action[1]:
                            print("-*-*- act(): we got a match!")
                            module_index = i
                            phase = phasei
                            raise Found
                        i += 1
            except Found:

                # remove the component
                #print("-*-*- act():  phase = "+str(phase)+", module_index = "+str(module_index))
                if phase and module_index > -1:
                    mod = self.mem.myMidca.remove_module(phase, module_index)
                    print "mod is "+str(mod)
                    self.prev_init_args = mod.get_init_args() # TODO: only works for modules implementing get_init_args
                    print"got init args: "+str(self.prev_init_args) 
                    is_success = mod_str not in map(lambda x: x.__class__.__name__, self.mem.myMidca.get_modules(phase))
                    if is_success: print("    Metareasoner removed "+mod_str) # report any actions metareasoner carried out
                    return is_success
        elif action[0] == "ADD-MODULE":
            if action[2] == "PyHopPlanner":
                #print("current directory: "+str(os.getcwd()))
                #print("str(dir(modules)) = "+str(dir(modules)))
                planningModuleInstance = importlib.import_module("midca.modules.planning")
                print("loaded planning module, it has following attributes: "+str(dir(planningModuleInstance)))
                # get the args used to init the old module and use them to init this one
                print "init args is "+str(self.prev_init_args)
                
                # **** BEGIN: MAGIC! Transform the args *****
                # This is where the real magic happens
                # Hardcoded for now
                # Very Important TODO but likely requires serious research effort
                from midca.domains.blocksworld.plan import methods
                working_methods = methods.declare_methods
                corrected_args = self.prev_init_args
                hardcoded_index_of_methods_arg = 2 # TODO: I know, so hacky, ahhh magic numbers
                corrected_args[2] = working_methods
                # **** END: MAGIC! Transform the args *****
                
                pyHopPlannerInstance = planningModuleInstance.PyHopPlanner(*corrected_args)
                self.mem.myMidca.runtime_append_module("Plan", pyHopPlannerInstance) # TODO: hardcoded knowledge of Plan phase
                is_success = "PyHopPlanner" in map(lambda x: x.__class__.__name__, self.mem.myMidca.get_modules("Plan"))
                if is_success: print("    Metareasoner added PyHopPlanner") # report any actions metareasoner carried out
                return is_success
            elif action[2] == "AsynchPyhopPlanner":
                #print("current directory: "+str(os.getcwd()))
                #print("str(dir(modules)) = "+str(dir(modules)))
                planningModuleInstance = importlib.import_module("midca.modules.planning")
                print("loaded asynchronous planning module, it has following attributes: "+str(dir(planningModuleInstance)))
                # get the args used to init the old module and use them to init this one
                #print "init args is "+str(self.prev_init_args)
                ###### HACK: hardcoded for now as a demo, this is because the initial broken
                # planner had different parameters for instantiating it, so we cant use the\
                # same prev_init_args, and thus changing them here
                from midca.modules._plan.asynch import operators_sr, methods_sr
                self.prev_init_args = [methods_sr.declare_methods, operators_sr.declare_ops]
                
                # **** BEGIN: MAGIC! Transform the args *****
                # This is where the real magic happens
                # Hardcoded for now
                # Very Important TODO but likely requires serious research effort
                #from midca.domains.blocksworld.plan import methods
                #working_methods = methods.declare_methods
                corrected_args = self.prev_init_args
                #hardcoded_index_of_methods_arg = 2 # TODO: I know, so hacky, ahhh magic numbers
                #corrected_args[2] = working_methods
                # **** END: MAGIC! Transform the args *****
                
                pyHopPlannerInstance = planningModuleInstance.AsynchPyhopPlanner(*corrected_args)
                self.mem.myMidca.runtime_append_module("Plan", pyHopPlannerInstance) # TODO: hardcoded knowledge of Plan phase
                is_success = "AsynchPyhopPlanner" in map(lambda x: x.__class__.__name__, self.mem.myMidca.get_modules("Plan"))
                if is_success: print("    Metareasoner added AsynchPyhopPlanner") # report any actions metareasoner carried out
                return is_success
        elif action[0] == "TRANSFORM-GOAL":
            # really: its going to have this meta plan by changing the things in orange - 
            # you don't monitor and look at the trace to see if the goal transformation achieved something
            # at the ground level, so then meta-evaluate can know if the metagoal succeeded. What if I made the
            # wrong transformation? 
            
            # specific to mortar and blocks world, not general
            # get the highest blocks
            goal_atoms = action[1]
            goal_args = map(lambda x: x.get_args(), goal_atoms)
            
            # figure out how much mortar we have using the world state
            num_available_mortar = 0
            for atom in map(str, self.mem.get("__world states")[-1].get_atoms()):
                if 'available(' in atom:
                    num_available_mortar +=1
                    if verbose >= 3: print("found available mortar: "+str(atom))
            
            # transform 'stable-on' to 'on' predicates ensuring there is enough mortar for each
            bottom_blks = []
            top_blks = []
            for a_tpl in goal_args:
                bottom_blks.append(a_tpl[1])
                top_blks.append(a_tpl[0])
            
            bottommost_blk = [b for b in bottom_blks if b not in top_blks][0]
            
            transformed_goal_atoms = []
            
            # transform goal atoms
            curr_bottom_blk = bottommost_blk
            while curr_bottom_blk in bottom_blks:
                i = bottom_blks.index(curr_bottom_blk)
                top_blks[i]
                if num_available_mortar > 0:
                    transformed_goal_atoms.append("stable-on,"+top_blks[i]+","+curr_bottom_blk)
                    num_available_mortar-=1
                else:
                    transformed_goal_atoms.append("on,"+top_blks[i]+","+curr_bottom_blk)
                curr_bottom_blk = top_blks[i]
            
            # now make actual MIDCA goal objects
            transformed_goals = []
            for atom_str in transformed_goal_atoms:
                vals = atom_str.split(",")
                transformed_goals.append(goals.Goal(*[vals[1],vals[2]], predicate = vals[0]))
             
            goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
            
            # do removal first
            for g in goal_atoms:
                goalGraph.remove(g)
            if verbose >= 2:
                # now display success statement
                print("Removed from the goal graph the previous goal:")
                for g in goal_atoms:
                    print("    "+str(g))
                    
            # now display transformed goal
            if verbose >= 2: 
                print("Goal has been transformed to:")
                for g in transformed_goals:
                    print("    "+str(g))
                    
            # now insert the new goal
            for g in transformed_goals:
                    goalGraph.insert(g)
            if verbose >= 2:
                print("Transformed goal has been inserted into goal graph.")
             
            return True
            #goals.Goal(*['A_','B_'], predicate = 'stable-on'),
            #       goals.Goal(*['C_','A_'], predicate = 'stable-on'),
            #       goals.Goal(*['D_','C_'], predicate = 'stable-on')
            
            # and go top down
            # first, get the heights

class MRSimpleControl1(base.BaseModule):
    midca = None
    '''
    def init(self, world):
        if not world:
            raise ValueError("world is None!")
        self.world = world
    '''

    def run(self, cycle, verbose = 2):
        plan = self.mem.get(self.mem.META_PLAN)
        failed = False
        if plan:
            for action in plan:
                if not self.act(action):
                    failed = True
                    if verbose >= 2: print("    MRSimpleControl failed to complete an action")
                    self.cleanup_and_reset()
            if not failed:
                self.cleanup_and_reset()
        else:
            if verbose >= 2: print("    No actions taken")



    def cleanup_and_reset(self):
        """
        Temporary function to remove goal and plan,
        should eventually be replaced by evaluate
         """
        self.mem.set(self.mem.META_PLAN, None)
        self.mem.set(self.mem.META_GOALS, None)
        self.mem.set(self.mem.META_CURR_GOAL, None)

    def act(self, action, verbose = 2):
        if action[0] == "REMOVE-MODULE":
            # find the component
            module_index = -1
            phase = None
            mod_str = ""
            class Found(Exception): pass # is this bad python? should this go at top of my file?
            try:
                for phasei in self.mem.myMidca.get_phases():
                    i = 0
                    for mod in self.mem.myMidca.get_modules(phasei):
                        mod_str = str(mod.__class__.__name__)
                        #print("-*-*- act():  mod = "+mod_str+", action[1] = "+str(action[1]))
                        if mod_str == action[1]:
                            #print("-*-*- act(): we got a match!")
                            module_index = i
                            phase = phasei
                            raise Found
                        i += 1
            except Found:

                # remove the component
                #print("-*-*- act():  phase = "+str(phase)+", module_index = "+str(module_index))
                if phase and module_index > -1:
                    self.mem.myMidca.remove_module(phase, module_index)
                    is_success = mod_str not in map(lambda x: x.__class__.__name__, self.mem.myMidca.get_modules(phase))
                    if is_success: print("    Metareasoner removed "+mod_str) # report any actions metareasoner carried out
                    return is_success
        elif action[0] == "ADD-MODULE":
            if action[2] == "PyHopPlanner":
                #print("current directory: "+str(os.getcwd()))
                planningModuleInstance = __import__("modules.planning")
                #print("loaded planning module, it has following attributes: "+str(dir(planningModuleInstance)))
                pyHopPlannerInstance = planningModuleInstance.planning.PyHopPlanner(True)
                self.mem.myMidca.runtime_append_module("Plan", pyHopPlannerInstance) # TODO: hardcoded knowledge of Plan phase
                is_success = "PyHopPlanner" in map(lambda x: x.__class__.__name__, self.mem.myMidca.get_modules("Plan"))
                if is_success: print("    Metareasoner added PyHopPlanner") # report any actions metareasoner carried out
                return is_success
        elif action[0] == "TRANSFORM-GOAL":
            # really: its going to have this meta plan by changing the things in orange - 
            # you don't monitor and look at the trace to see if the goal transformation achieved something
            # at the ground level, so then meta-evaluate can know if the metagoal succeeded. What if I made the
            # wrong transformation? 
            #print(self.world.cltree.rootnode.predicate)
            # specific to mortar and blocks world, not general
            # get the highest blocks
	    # goal_check is the copy of goal graph intended to check the goalgraph goals and delete the goals in the goal graph
	    goal_check = copy.deepcopy(action[1])
            goal_atoms = action[1]
            goal_args = map(lambda x: x.get_args(), goal_atoms)
	    mem = self.mem
	    # get the world
            world = self.mem.myMidca.midca.world
	    # initialize the resources for goal transform
	    goaltransform.resources(mem)
	    transformed_goals = []
	    goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
	    for i in range(0,len(goal_atoms)):
		trans_goal=goaltransform.choose(mem,goal_atoms[i])
		transformed_goals.append(trans_goal)				
		world.cltree['checked'] = []
	
            # do removal first
            for g in goal_check:
                goalGraph.remove(g)

            if verbose >= 2:
                # now display success statement
                print("Removed from the goal graph the previous goal:")
                for g in goal_check:
                    print("    "+str(g))
                    
            # now display transformed goal
            if verbose >= 2: 
                print("Goal has been transformed to:")
                for g in transformed_goals:
                    print("    "+str(g))

                    
            # now insert the new goal
            for g in transformed_goals:
                    goalGraph.insert(g)

	    self.mem.set(self.mem.CURRENT_GOALS, transformed_goals)
            if verbose >= 2:
                print("Transformed goal has been inserted into goal graph.")
            
            return True
            #goals.Goal(*['A_','B_'], predicate = 'stable-on'),
            #       goals.Goal(*['C_','A_'], predicate = 'stable-on'),
            #       goals.Goal(*['D_','C_'], predicate = 'stable-on')
            
            # and go top down
            # first, get the heights

            

