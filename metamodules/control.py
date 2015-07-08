import os
from MIDCA import base
class MRSimpleControl(base.BaseModule):
    midca = None

    def run(self, cycle, verbose = 2):
        plan = self.mem.get(self.mem.META_PLAN)
        if plan:
            for action in plan:
                self.act(action)

    def act(self, action, verbose = 0):
        if action[0] == "REMOVE-MODULE":
            # find the component
            module_index = -1
            phase = None
            mod_str = ""
            class Found(Exception): pass # is this bad python? should this go at top of my file?
            try:
                for phasei in self.midca.get_phases():
                    i = 0
                    for mod in self.midca.get_modules(phasei):
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
                    self.midca.remove_module(phase, module_index)
                    is_success = mod_str not in map(lambda x: x.__class__.__name__, self.midca.get_modules(phase))
                    if is_success: print("Metareasoner removed "+mod_str) # report any actions metareasoner carried out
        elif action[0] == "ADD-MODULE":
            if action[2] == "PyHopPlanner":
                print("current directory: "+str(os.getcwd()))
                planningModuleInstance = __import__("modules.planning")
                print("loaded planning module, it has following attributes: "+str(dir(planningModuleInstance)))
                pyHopPlannerInstance = planningModuleInstance.planning.PyHopPlanner(True)
                self.midca.runtime_append_module("Plan", pyHopPlannerInstance) # TODO: hardcoded knowledge of Plan phase
                is_success = "PyHopPlanner" in map(lambda x: x.__class__.__name__, self.midca.get_modules("Plan"))
                if is_success: print("Metareasoner added PyHopPlanner") # report any actions metareasoner carried out

