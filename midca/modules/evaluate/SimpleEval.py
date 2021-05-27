from midca import base
from midca import midcatime
import copy

class SimpleEval(base.BaseModule):

    def run(self, cycle, verbose = 2):
        world = self.mem.get(self.mem.STATES)[-1]
        try:
            goals = self.mem.get(self.mem.CURRENT_GOALS)[-1]
        except:
            goals = []

        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("WORLD", copy.deepcopy(world))
            trace.add_data("GOALS", copy.deepcopy(goals))

        goals_changed = False # for trace
        if goals:
            for goal in goals:
                try:
                    achieved = world.atom_true(world.midcaGoalAsAtom(goal))
                    if 'negate' in goal and goal['negate']:
                        achieved = not achieved
                    if not achieved:
                        if verbose >= 2:
                            print("Not all goals achieved;", goal, "is not true.")
                        return
                except ValueError:
                    if verbose >= 1:
                        print("Could not test goal", goal, ". It does not seem to be a valid world state")
                    return
            if verbose >= 1:
                print("All current goals achieved. Removing them from goal graph")
            goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
            for goal in goals:
                goalGraph.remove(goal)
                if trace: trace.add_data("REMOVED GOAL", goal)
                goals_changed = True
            numPlans = len(goalGraph.plans)
            goalGraph.removeOldPlans()
            newNumPlans = len(goalGraph.plans)
            if numPlans != newNumPlans and verbose >= 1:
                print("removing", numPlans - newNumPlans, "plans that no longer apply.")
                goals_changed = True
            del goals[-1]
            self.mem.set(self.mem.CURRENT_GOALS,goals)
        else:
            if verbose >= 2:
                print("No current goals. Skipping eval")

        if trace and goals_changed: trace.add_data("GOALS",goals)

class SimpleEval2(base.BaseModule):

    def run(self, cycle, verbose = 2):
        world = self.mem.get(self.mem.STATES)[-1]
        try:
            goals = self.mem.get(self.mem.CURRENT_GOALS)[-1]
        except:
            goals = []

        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("WORLD", copy.deepcopy(world))
            trace.add_data("GOALS", copy.deepcopy(goals))

        goals_changed = False # for trace
        if goals:
            for goal in goals:
                try:
                    achieved = world.atom_true(world.midcaGoalAsAtom(goal))
                    if 'negate' in goal and goal['negate']:
                        achieved = not achieved
                    if achieved:
                        print("achieved")
                        score = self.mem.get(self.mem.DELIVERED)
                        lastGoals = self.mem.get(LAST_SCORED_GOAL)
                        if not lastGoals:
                            self.mem.set(LAST_SCORED_GOAL, [goal])
                            self.mem.set(self.mem.DELIVERED, 1)
#                             self.generate_thief_file(goal, 5)
                        elif not (goal in lastGoals):
                            self.mem.add(LAST_SCORED_GOAL, goal)
                            self.mem.set(self.mem.DELIVERED, score+1)


                        print(str(self.mem.get(self.mem.DELIVERED)))
                    if not achieved:
                        if verbose >= 2:
                            print("Not all goals achieved;", goal, "is not true.")
                        return
                except ValueError:
                    if verbose >= 1:
                        print("Could not test goal", goal, ". It does not seem to be a valid world state")
                    return
            if verbose >= 1:
                print("All current goals achieved. Removing them from goal graph")
            goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
            for goal in goals:
                goalGraph.remove(goal)
                if trace: trace.add_data("REMOVED GOAL", goal)
                goals_changed = True
            numPlans = len(goalGraph.plans)
            goalGraph.removeOldPlans()
            newNumPlans = len(goalGraph.plans)
            if numPlans != newNumPlans and verbose >= 1:
                print("removing", numPlans - newNumPlans, "plans that no longer apply.")
                goals_changed = True
            del goals[-1]
            self.mem.set(self.mem.CURRENT_GOALS,goals)
        else:
            if verbose >= 2:
                print("No current goals. Skipping eval")

        if trace and goals_changed: trace.add_data("GOALS",goals)


    def generate_thief_file(self,goal, stolen_number):
        args = [str(arg) for arg in goal.args]
        current_w = args[2]
        graph = self.mem.get(self.mem.GOAL_GRAPH)
        goals = graph.getAllGoals()

        other_w_goals = []

        for g in goals:
            args = [str(arg) for arg in g.args]
            if args[2] != current_w:
                other_w_goals.append(g)
#         other_w_goals =  filter(lambda g: str(g.args[2]) != current_w, goals.copy())
        randomgoals = random.sample(other_w_goals, stolen_number)

#         thisDir =  "C:/Users/Zohreh/git/midca/modules/_plan/jShop"
        thisDir = os.path.dirname(os.path.realpath(__file__))
        thief_file = thisDir + "/theif.txt"

        f = open(thief_file, 'w')

        for g in randomgoals:
            f.write("obj-at" + " " + str(g.args[0]) + " " + str(g.args[2])+"\n")

class SimpleEval_Restaurant(base.BaseModule):

    def run(self, cycle, verbose = 2):
        world = self.mem.get(self.mem.STATES)[-1]
        try:
            goals = self.mem.get(self.mem.CURRENT_GOALS)
        except KeyError:
            goals = []

        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("WORLD", copy.deepcopy(world))
            trace.add_data("GOALS", copy.deepcopy(goals))

        goals_changed = False # for trace

        # if the time from memory is 0 then remove all goals from the goal graph
        goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
        money = self.mem.get(self.mem.MONEY)
        if money == 0:
            print("Current Goals and Goals in Goal Graph are Removed Due to Insuffecient money")
            for goal in goalGraph.getUnrestrictedGoals():
                goalGraph.remove(goal)
            self.mem.set(self.mem.CURRENT_GOALS , [])
            self.mem.set(self.mem.GOAL_GRAPH,goalGraph)
            goals = []

        if goals:
            for goal in goals:
                try:
                    achieved = world.atom_true(world.midcaGoalAsAtom(goal))
                    if 'negate' in goal and goal['negate']:
                        achieved = not achieved
                    if not achieved:
                        if verbose >= 2:
                            print("Not all goals achieved;", goal, "is not true.")
                        return
                except ValueError:
                    if verbose >= 1:
                        print("Could not test goal", goal, ". It does not seem to be a valid world state")
                    return
            if verbose >= 1:
                print("All current goals achieved. Removing them from goal graph")
                self.mem.set(self.mem.CURRENT_GOALS, None)
            goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
            for goal in goals:
                goalGraph.remove(goal)
                if trace: trace.add_data("REMOVED GOAL", goal)
                goals_changed = True
            numPlans = len(goalGraph.plans)
            goalGraph.removeOldPlans()
            newNumPlans = len(goalGraph.plans)
            if numPlans != newNumPlans and verbose >= 1:
                print("removing", numPlans - newNumPlans, "plans that no longer apply.")
                goals_changed = True
        else:
            if verbose >= 2:
                print("No current goals. Skipping eval")

        if trace and goals_changed: trace.add_data("GOALS",goals)

class SimpleEval_construction(base.BaseModule):

    def run(self, cycle, verbose = 2):
        world = self.mem.get(self.mem.STATES)[-1]



        try:
            goals = self.mem.get(self.mem.CURRENT_GOALS)
        except KeyError:
            goals = []

        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("WORLD", copy.deepcopy(world))
            trace.add_data("GOALS", copy.deepcopy(goals))

        # this variable is to skip one eval phase, when the building gets completed
        try:
            self.skip
        except AttributeError:
            self.skip = False

        goals_changed = False # for trace

        goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
        time = self.mem.get(self.mem.TIME_CONSTRUCTION)

        if time == 0:
            print("Current Goals and Goals in Goal Graph are Removed Due to Insuffecient Time")
            for goal in goalGraph.getUnrestrictedGoals():
                goalGraph.remove(goal)
            goalGraph.removeOldPlans()
            self.mem.set(self.mem.CURRENT_GOALS , [])
            self.mem.set(self.mem.REJECTED_GOALS , None)
            self.mem.set(self.mem.GOAL_GRAPH,goalGraph)
            goals = []


        rejected_goals = self.mem.get(self.mem.REJECTED_GOALS)

        if rejected_goals :
            print("Current Goals and Goals in Goal Graph are Removed Due to Insuffecient Time")
            for goal in goalGraph.getUnrestrictedGoals():
                goalGraph.remove(goal)
            self.mem.set(self.mem.CURRENT_GOALS , [])
            goals_changed = True
            self.mem.set(self.mem.REJECTED_GOALS , None)
            self.mem.set(self.mem.GOAL_GRAPH,goalGraph)
            goals=[]



        if goals:
            for goal in goals:
                try:
                    achieved = world.atom_true(world.midcaGoalAsAtom(goal))
                    if 'negate' in goal and goal['negate']:
                        achieved = not achieved
                    if not achieved:
                        if verbose >= 2:
                            print("Not all goals achieved;", goal, "is not true.")
                        return
                except ValueError:
                    if verbose >= 1:
                        print("Could not test goal", goal, ". It does not seem to be a valid world state")
                    return

            if self.skip == False:
                if verbose >= 1:
                    print("Skipping one phase")
                self.skip = True
                return
            if self.skip ==True:
                del self.skip

            if verbose >= 1:
                print("All current goals achieved. Removing them from goal graph")
                self.mem.set(self.mem.CURRENT_GOALS, None)
            goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
            for goal in goals:
                goalGraph.remove(goal)
                if trace: trace.add_data("REMOVED GOAL", goal)
                goals_changed = True
            numPlans = len(goalGraph.plans)
            goalGraph.removeOldPlans()
            newNumPlans = len(goalGraph.plans)
            if numPlans != newNumPlans and verbose >= 1:
                print("removing", numPlans - newNumPlans, "plans that no longer apply.")
                goals_changed = True
        else:
            if verbose >= 2:
                print("No current goals. Skipping eval")

        if trace and goals_changed: trace.add_data("GOALS",goals)
