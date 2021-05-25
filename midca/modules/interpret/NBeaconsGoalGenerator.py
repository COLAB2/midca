from midca import goals, base
from midca import midcatime
from ._goalgen import tf_fire
from ._goalgen import tf_3_scen
from midca.domains.logistics import deliverstate
from midca.domains.blocksworld import blockstate
from midca.worldsim import stateread
import copy,csv
import random
from midca.modules.monitors import Monitor


class NBeaconsGoalGenerator(base.BaseModule):
    '''
    MIDCA module for the nbeacons domain. Generates a goal to activate 3 different
    beacons in the domain.
    '''

    def __init__(self, numbeacons=3, goalList=[]):
        self.numbeacons = numbeacons
        self.goalList = goalList
        self.currGoalIndex = 0

    def activateGoalsExist(self):
        graph = self.mem.get(self.mem.GOAL_GRAPH)
        for goal in graph.getAllGoals():
            if goal['predicate'] == "activated":
                return True
        return False

    def generate_new_goals(self):
        if self.currGoalIndex < len(self.goalList):
            curr_goal = self.goalList[self.currGoalIndex]
            #if self.verbose >= 1: print "inserting goal "+str(curr_goal)
            self.currGoalIndex+=1
            return [curr_goal]
        if self.currGoalIndex == len(self.goalList):
            print("No more goals...")
            self.currGoalIndex+=1
        return []

        # this is a safety check to make sure experiments are running correctly.
        # if running manual (like running from examples/nbeacons...agentx.py remove this line
        raise Exception("randomly inserting goals, shouldn't be here when running from nbeacons_experiment_1.py")
        world = self.mem.get(self.mem.STATES)[-1]
        goal_b_ids = []
        # get all beacon ids
        unactivated_b_ids = []
        for obj in world.get_possible_objects("",""):
            # test if a beacon id
            if str(obj).startswith("B"):
                # now test to see if it's activated
                if not world.is_true('activated',[str(obj)]):
                    unactivated_b_ids.append(str(obj))

        if len(unactivated_b_ids) == 0:
            if self.verbose >= 1: print("All beacons are activated. No activation goals will be generated.")
            return []

        num_chosen_beacons = 0
        while len(unactivated_b_ids) > 0 and num_chosen_beacons < self.numbeacons:
            b = random.choice(unactivated_b_ids)
            unactivated_b_ids.remove(b)
            goal_b_ids.append(b)
            num_chosen_beacons+=1

        # turn into goals
        new_goals = [goals.Goal(str(x), predicate = "activated") for x in goal_b_ids]

        return new_goals

    def run(self, cycle, verbose = 2):
        self.verbose = verbose
        if self.activateGoalsExist():
            if self.verbose >=1: print("MIDCA already has an activation goal. Skipping goal generation")
            return
        else:
            new_goals = self.generate_new_goals()
            for g in new_goals:
                self.mem.get(self.mem.GOAL_GRAPH).insert(g)
                if self.verbose >= 1: print(("Inserted goal "+str(g)))

class SimpleNBeaconsGoalManager(base.BaseModule):
    '''
    MIDCA module for the nbeacons domain. Using the discrepancies and explanations from
    note.DiscrepancyDetect and assess.SimpleExplain, this module will create new goals and
    insert / remove goals from the goal graph.
    '''

    def run(self, cycle, verbose=2):
        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
        self.verbose = verbose
        # get discrepancies
        discrep = self.mem.get(self.mem.DISCREPANCY)
        # if discrepancy, get explanation
        if discrep and (len(discrep[0]) > 0 or len(discrep[1]) > 0):
            # first remove old
            # go ahead and remove old plans for any goals the agent has
            # refresh the goals to trigger replanning
            goalgraph = self.mem.get(self.mem.GOAL_GRAPH)
            curr_goals = self.mem.get(self.mem.CURRENT_GOALS)

            if type(curr_goals) is not list:
                curr_goals = [curr_goals]
            for goal in curr_goals:
                # get any plans associated with this goal, and remove them
                plan = goalgraph.getMatchingPlan([goal])
                if plan:
                    goalgraph.removePlan(plan)
                    if self.verbose >= 1: print("Just removed a plan for goal " +str(goal))

            #print "aware of actual discrepancy, retrieving explanation"
            explain_exists = self.mem.get(self.mem.EXPLANATION)
            explanation = self.mem.get(self.mem.EXPLANATION_VAL)
            if explain_exists:

                # now do stuff based on explanation
                if 'stuck' in explanation:
                    # remove current goal from goal graph
                    # insert goal to become free
                    # The priority will automatically be shifted to
                    goalgraph = self.mem.get(self.mem.GOAL_GRAPH)
                    free_goal = goals.Goal('Curiosity', predicate = "free")
                    goalgraph.insert(free_goal)
                    if self.verbose >= 1: print("Just inserted goal "+str(free_goal))
                    return
                else: #if 'wind' in explanation:
                    # do nothing for other explanations, this will just lead to replanning
                    return
            else:
                if self.verbose >= 1: print("No explanation, old plans removed, but no goal management actions")
                return
