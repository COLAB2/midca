from MIDCA import base
import copy

class SimpleIntend(base.BaseModule):

    def run(self, cycle, verbose = 2):
        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("GOALGRAPH",copy.deepcopy(self.mem.GOAL_GRAPH))

        goalGraph = self.mem.get(self.mem.GOAL_GRAPH)

        if not goalGraph:
            if verbose >= 1:
                print "Goal graph not initialized. Intend will do nothing."
            return
        goals = goalGraph.getUnrestrictedGoals()
        goals_selected = []
        # special code for NBeacons, need to change for later
        exists_free_goal = False
        free_goal = None
        for g in goals:
            if 'free' in str(g):
                exists_free_goal = True
                free_goal = g
        
        # activation goals: just pick one
        
        if free_goal:
            goals_selected = [free_goal]
            self.mem.set(self.mem.CURRENT_GOALS,goals_selected )
        else:
            if len(goals) > 1:
                # just randomly pick the first one
                goals_selected = [goals[0]]
                self.mem.set(self.mem.CURRENT_GOALS,goals_selected )
            else:
                goals_selected = goals
                self.mem.set(self.mem.CURRENT_GOALS, goals_selected)
        if trace:
            trace.add_data("GOALS",goals)

        if not goals:
            if verbose >= 2:
                print "No goals selected."
        else:
            if verbose >= 2:
                print "Selecting goal(s):",
                for goal in goals_selected:
                    print goal,
                print
