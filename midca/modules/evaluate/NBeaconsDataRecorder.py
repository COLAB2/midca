from midca import base
from midca import midcatime
import copy


LAST_SCORED_GOAL = "Last Scored Goal"
SCORE = "Score"

class NBeaconsDataRecorder:
    '''
    Records data in the NBeacons Domain including:
    - number of goals achieved
    - number of actions executed
    - number of times replanning happened
    '''

    def __init__(self):
        pass

    def init(self, world, mem):
        self.mem = mem
        self.mem.set(LAST_SCORED_GOAL, None)
        self.mem.set(self.mem.GOALS_ACTIONS_ACHIEVED, [(0,'',0)])
        self.mem.set(self.mem.ACTIONS_EXECUTED, 0)

    def get_activation_goal(self):
        activation_goals = []

        # safety check
        if not self.mem.get(self.mem.CURRENT_GOALS)[-1]:
            return activation_goals

        for goal in self.mem.get(self.mem.CURRENT_GOALS)[-1]:
            if 'predicate' in goal and goal['predicate'] == 'activated':
                activation_goals.append(goal)
        return activation_goals

    def newrun(self, cycle, verbose = 2):

        lastGoal = self.mem.get(LAST_SCORED_GOAL)
        currentGoal = self.get_activation_goal()
        all_achieved = True
        if not currentGoal or lastGoal == currentGoal:
            return #no goal or goal already scored
        try:
            all_achieved = True
            at_least_one_achieved = False
            for goal in currentGoal:
                print("checking goal "+str(goal))
                achieved = self.world.atom_true(self.world.midcaGoalAsAtom(goal))
                if not achieved:
                    all_achieved = False
                    break
                else:
                    at_least_one_achieved = True
        except Exception:
            print("unable to check goal", currentGoal, ". skipping scoring")

        if at_least_one_achieved and all_achieved:
            print(("All goals "+str(list(map(str,currentGoal)))+" were achieved"))
            goal_action_pairs = self.mem.get(self.mem.GOALS_ACTIONS_ACHIEVED)
            last_pair = goal_action_pairs[-1]
            actions_executed_thus_far = self.mem.get(self.mem.ACTIONS_EXECUTED)
            self.mem.set(self.mem.GOALS_ACTIONS_ACHIEVED, goal_action_pairs+[(last_pair[0]+1,actions_executed_thus_far)])


        if not all_achieved:
            return
        self.mem.set(LAST_SCORED_GOAL, currentGoal)

    def run(self,cycle,verbose=2):
        self.verbose = verbose
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
        all_goals_achieved = True
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
                    all_goals_achieved = False
                    if verbose >= 1:
                        print("Could not test goal", goal, ". It does not seem to be a valid world state")
                    return
            if verbose >= 1:
                print("All current goals achieved. Removing them from goal graph")

            goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
            if all_goals_achieved:
                # remove goals
                for goal in goals:
                    if 'activated' in str(goal):
                        # incrememnt goals achieved counter
                        goals_achieved = self.mem.get(self.mem.GOALS_ACHIEVED)
                        if goals_achieved is None:
                            goals_achieved = 0
                        goals_achieved +=1
                        self.mem.set(self.mem.GOALS_ACHIEVED, goals_achieved)
                        # store goal achieved and actions executed
                        goal_action_pairs = self.mem.get(self.mem.GOALS_ACTIONS_ACHIEVED)
                        actions_executed_thus_far = self.mem.get(self.mem.ACTIONS_EXECUTED)
                        if self.verbose >= 1: print(" just added goals actions pair: " +str((goals_achieved,str(goal.args[0]),actions_executed_thus_far)))
                        self.mem.set(self.mem.GOALS_ACTIONS_ACHIEVED, goal_action_pairs+[(goals_achieved,str(goal.args[0]),actions_executed_thus_far)])

                    goalGraph.remove(goal)
                    if trace: trace.add_data("REMOVED GOAL", goal)
                    goals_changed = True
            numPlans = len(goalGraph.plans)
            goalGraph.removeOldPlans()
            newNumPlans = len(goalGraph.plans)
            if numPlans != newNumPlans and verbose >= 1:
                if self.verbose >= 1: print("removing", numPlans - newNumPlans, "plans that no longer apply.")
                goals_changed = True
            del goals[-1]
            self.mem.set(self.mem.CURRENT_GOALS,goals)
        else:
            if verbose >= 2:
                print("No current goals. Skipping eval")

        if trace and goals_changed: trace.add_data("GOALS",goals)


    def oldrun(self,cycle,verbose=2):
        world = self.mem.get(self.mem.STATES)[-1]
        currentPlan = self.mem.get(self.mem.CURR_PLAN)
        if currentPlan:
            currentgoals = currentPlan.goals
        else:
            currentgoals = None
        accomplished = True
        if currentgoals:
            if not hasattr(currentgoals, "__iter__"):
                currentgoals = [currentgoals]
            for currentgoal in currentgoals:
                accomplishedthis = False
                if currentgoal.goaltype == "activated":
                    accomplishedthis = world.is_true("activated", [arg.id for arg in currentgoal.goalargs])
                    print(("accomplishedthis = "+str(accomplishedthis)+" for activated("+str([arg.id for arg in currentgoal.goalargs])))
                accomplished = accomplished and accomplishedthis

            if verbose >= 1:
                s = "current goals: " + "".join([str(currentgoal) + " " for currentgoal in currentgoals]) + " have "
                if not accomplished:
                    s += "not "
                s += "been accomplished."
                print(s)
        else:
            if verbose >= 2:
                print("No current goal. Skipping Eval.")
#         if world.is_true("on", ["D_", "C_"]):
#             if verbose >= 2:
#                 print "Tall Tower completed"
#             self.towersFinished += 1
#             self.score += (4 - self.num_fires(world)) * NORMAL_BLOCK_VAL + self.num_fires(world) * ON_FIRE_BLOCK_VAL
#             if self.restartFires:
#                 self.put_out_fires(verbose)
#         elif world.is_true("on", ["D_", "B_"]):
#             if verbose >= 2:
#                 print "Short Tower completed"
#             self.towersFinished += 1
#             self.score += (3 - self.num_fires_small(world)) * NORMAL_BLOCK_VAL + self.num_fires_small(world) * ON_FIRE_BLOCK_VAL
#             if self.restartFires:
#                 self.put_out_fires(verbose)
#         if verbose >= 2:
#             print "Total towers built:", self.towersFinished
#             print "Total fire turns:", self.fireturns
#             print "Total score:", self.score
        #self.mem._update(self.memKeys.MEM_GOAL_COMPLETED, accomplished)
        if accomplished and currentPlan:
            self.mem.set(self.mem.GOALS_ACHIEVED, 1+self.mem.get(self.mem.GOALS_ACHIEVED))
            if verbose >= 2:
                print("Goal completed for current plan. Removing all intended goals and pending plans for this goal.")
            #self.mem._update(self.memKeys.MEM_OLD_PLANS, currentPlan)
            self.mem.set(self.mem.CURR_PLAN, None)
            self.mem.get(self.mem.MEM_PLANS).remove_goals(currentgoals)
            self.mem.get(self.mem.MEM_GOALS).remove_goal_set(currentgoals)
