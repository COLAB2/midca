import os
from MIDCA import base, goals

class MRSimpleControl(base.BaseModule):
    midca = None

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
            

