from midca.modules._plan.asynch import asynch
from midca import base
import copy


class NBeaconsSimpleAct(base.BaseModule):

    '''
    MIDCA module that selects the plan, if any, that achieves the most current goals, then selects the next action from that plan. The selected action is stored in a two-dimensional array in mem[mem.ACTIONS], where mem[mem.ACTIONS][x][y] returns the yth action to be taken at time step x. So mem[mem.ACTIONS][-1][0] is the last action selected. Note that this will throw an index error if no action was selected.
    To have MIDCA perform multiple actions in one cycle, simple add several actions to mem[mem.ACTIONS][-1]. So mem[mem.ACTIONS][-1][0] is the first action taken, mem[mem.ACTIONS][-1][1] is the second, etc.
    '''

    def get_first_plan(self, goals):
        goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
        plans = goalGraph.allMatchingPlans(goals)
        for p in plans:
            if p.finished():
                goalGraph.removePlan(p)
                if self.verbose >= 1:
                    print("Just removed finished plan ")
                    for ps in p:
                        print("  "+str(ps))
            else:
                return p
        if self.verbose >= 1: print("Could not find an unfinished plan in get_first_plan() for goals "+str(goals))
        return None

    def run(self, cycle, verbose = 2):
        self.verbose = verbose
        max_plan_print_size = 10
        world = self.mem.get(self.mem.STATES)[-1]
        try:
            goals = self.mem.get(self.mem.CURRENT_GOALS)[-1]
        except:
            goals = []
        plan = self.get_first_plan(goals)

        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("WORLD", copy.deepcopy(world))
            trace.add_data("GOALS", copy.deepcopy(goals))
            trace.add_data("PLAN", copy.deepcopy(plan))

        if plan != None:
            action = plan.get_next_step()
            if not action:
                if verbose >= 1:
                    print("Plan to achieve goals has already been completed. Taking no action.")
                self.mem.add(self.mem.ACTIONS, [])
            else:
                if verbose == 1:
                    print("Action selected:", action)
                elif verbose >= 2:
                    if len(plan) > max_plan_print_size:
                        # print just the next 3 actions of the plan
                        print("Selected action", action, "from plan:\n")
                        if verbose >= 3:
                            for a in plan:
                                if action == a:
                                    print("   *"+str(a))
                                else:
                                    print("  "+str(a))
                    else:
                        # print the whole plan
                        print("Selected action", action, "from plan:\n")
                        for a in plan:
                            if action == a:
                                print("   *"+str(a))
                            else:
                                print("  "+str(a))

                self.mem.add(self.mem.ACTIONS, [action])
                plan.advance()

                if trace: trace.add_data("ACTION", action)
        else:
            if verbose >= 1:
                print("MIDCA will not select an action this cycle.")
            self.mem.add(self.mem.ACTIONS, [])

            if trace: trace.add_data("ACTION", None)
