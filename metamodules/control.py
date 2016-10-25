import os
from MIDCA import base
class MRSimpleControl(base.BaseModule):
    midca = None

    def run(self, cycle, verbose = 2):
        self.verbose = verbose
        plan = self.mem.get(self.mem.META_PLAN)
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

    def act(self, action, verbose = 0):
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

