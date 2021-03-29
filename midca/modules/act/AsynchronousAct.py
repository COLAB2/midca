from midca.modules._plan.asynch import asynch
from midca import base
import copy


class AsynchronousAct(base.BaseModule):

    '''
    MIDCA module that "executes" plans in which the individual actions will be conducted
    asynchronously. This was originally designed to allow MIDCA to work as a robot
    controller in communication with ROS sensor and effector nodes.
    '''

    def run(self, cycle, verbose = 2):
        self.verbose = verbose
        try:
            goals = self.mem.get(self.mem.CURRENT_GOALS)[-1]
        except:
            goals = []
        if not goals:
            if verbose >= 2:
                print("No Active goals. Act phase will do nothing")
            return

        try:
            plan = self.mem.get(self.mem.GOAL_GRAPH).getMatchingPlan(goals)
        except:
            if verbose >= 1:
                print("Error loading plan. Skipping act phase.")
            return

        if not plan:
            if verbose > 2:
                print("No current plan. Skipping Act phase")
            return
        i = 0
        if plan.finished():
            print("Plan", plan, "has already been completed")
            return
        #ideally MIDCA should check for other valid plans, but for now it doesn't.

        while i < len(plan):
            action = plan[i]
            try:
                if action.status != asynch.FAILED and action.status != asynch.COMPLETE:
                    completed = action.check_complete()
                    if completed:
                        if verbose >= 2:
                            print("Action", action, "completed")
            except AttributeError:
                if verbose >= 1:
                    print("Action", action, "Does not seem to have a valid check_complete() ", end=' ')
                    "method. Therefore MIDCA cannot execute it."
                    action.status = asynch.FAILED
            try:
                if action.status == asynch.NOT_STARTED:
                    if verbose >= 2:
                        print("Beginning action execution for", action)
                    action.execute()
            except AttributeError:
                if verbose >= 1:
                    print("Action", action, "Does not seem to have a valid execute() ", end=' ')
                    "method. Therefore MIDCA cannot execute it"
                    action.status = asynch.FAILED
            if action.status == asynch.COMPLETE:
                i += 1
            elif not action.blocks:
                i += 1
            else:
                break

