from midca import base

class MRSimpleIntend(base.BaseModule):

    def run(self, cycle, verbose = 2):
        goals = self.mem.get(self.mem.META_GOALS)
        curr_goal = self.mem.get(self.mem.META_CURR_GOAL)
        if curr_goal:
            #print("    Already have a curr goal: "+str(curr_goal)+" ... no goal change")
            return
        if goals and len(goals) > 0:
            self.mem.set(self.mem.META_CURR_GOAL, goals[0])
            if verbose >= 2: print("    Selected goal: "+str(self.mem.get(self.mem.META_CURR_GOAL)))
        else:
            if verbose >= 2: print("    No goals to select from")
