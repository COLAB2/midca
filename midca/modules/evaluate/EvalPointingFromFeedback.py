from midca import base
from midca import midcatime
import copy
try:
    from midca.examples._gazebo_baxter import halo_color
except ImportError:
    pass

class EvalPointingFromFeedback(base.BaseModule):
    def run(self, cycle, verbose = 2):
        try:
            goals = self.mem.get(self.mem.CURRENT_GOALS)[-1]
        except:
            goals = []
        if not goals:
            if verbose >= 2:
                print("No current goals. Skipping eval")
        else:
            goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
            plan = goalGraph.getMatchingPlan(goals)
            if not plan:
                if verbose >= 2:
                    print("No plan found that achieves all current goals. ", end=' ')
                    "Skipping eval based on plan completion"
            else:
                if plan.finished():
                    try:
                        hallo = halo_color.HaloLed()
                        # set hallo to green as a sign for monitors
                        hallo.setGreen()
                    except:
                        pass
                    finally:
                        print("robot stuff")
                    if verbose >= 1:
                        print("Plan:", plan, "complete. Removing its goals")
                    for goal in plan.goals:
                        goalGraph.remove(goal)
                    numPlans = len(goalGraph.plans)
                    goalGraph.removeOldPlans()
                    newNumPlans = len(goalGraph.plans)
                    if numPlans != newNumPlans and verbose >= 1:
                        print("removing", numPlans - newNumPlans, end=' ')
                        "plans that no longer apply."
                    del goals[-1]
                    self.mem.set(self.mem.CURRENT_GOALS,goals)
                else:
                    if verbose >= 2:
                        print("Plan:", plan, "not complete")
