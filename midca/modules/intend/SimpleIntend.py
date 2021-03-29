from midca import base,midcatime
import copy,itertools,operator
import random
import time as ctime

class SimpleIntend(base.BaseModule):

    def run(self, cycle, verbose = 2):
        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("GOALGRAPH",copy.deepcopy(self.mem.GOAL_GRAPH))

        goalGraph = self.mem.get(self.mem.GOAL_GRAPH)

        if not goalGraph:
            if verbose >= 1:
                print("Goal graph not initialized. Intend will do nothing.")
            return
        # get all the goals from the root of the goal graph
        goals = goalGraph.getUnrestrictedGoals()

        if not goals:
            if verbose >= 1:
                print("No Goals in Goal graph. Intend will do nothing.")
            return

        # take the first goal
        goals = [goals[0]]
        # add it to the current goal in memory

        # current goals as a stack
        if self.mem.get(self.mem.CURRENT_GOALS) :
            current_goals = self.mem.get(self.mem.CURRENT_GOALS)
            if not current_goals[-1] == goals:
                current_goals.append(goals)
                self.mem.set(self.mem.CURRENT_GOALS, current_goals)
            else:
                goals = []
        else:
            self.mem.set(self.mem.CURRENT_GOALS, [goals])

        if trace:
            trace.add_data("GOALS",goals)

        if not goals:
            if verbose >= 2:
                print("No goals selected.")
        else:
            if verbose >= 2:
                print("Selecting goal(s):", end=' ')
                for goal in goals:
                    print(goal, end=' ')
                print()
