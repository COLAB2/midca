from midca import base


class MRfastdownPlanner(base.BaseModule):
    #goals_to_plans = {"SWAP-MODULE": [["REMOVE-MODULE", "?x"], ["ADD-MODULE", "?p", "?x"]],
     #                 "ACHIEVEABLE-GOAL": [["TRANSFORM-GOAL", "?g"]]}
    #goals_to_plans = {"startfoil spray"}

    def __init__(self, domainfile,statefile,using_baxter=False):
        self.using_baxter = using_baxter
        self.domainfile = domainfile
        self.statefile = statefile

    def run(self, cycle, verbose=2):
        self.verbose = verbose
        curr_goal = self.mem.get(self.mem.META_CURR_GOAL)
        if not curr_goal:
            if self.verbose >= 1: print("    No plan generated")
            return
        else:
            #plan = self.plan_for_goal(curr_goal)
            #plan= self.goals_to_plans
            import os, subprocess
            import os
            from midca.modules._plan.fastdown import fast_downward
            if (self.mem.DISCREPANCY is True):
                #os.chdir('/home/zahid/Downloads/midca-update-to-python36/midca/modules/_plan/fastdown')
                #running fastdownward planner with meta domain file and state file
                try:
                    fast_downward.plan(self.domainfile,self.statefile, "--search", "lazy_greedy([ff])")
                except:
                    pass
                f = open("sas_plan", "r")
                ffplan = f.read().split()
                for i in range(len(ffplan)):
                    ffplan[i] = ffplan[i].replace("(", "")
                    ffplan[i] = ffplan[i].replace(")", "")
                ffplan = ffplan[:-6]
                ffPlan1=[]
                for i in range(len(ffplan)):
                    if (ffplan[i] == "run-foil"):
                        # print(ffplan)
                        ffPlan1.append(tuple([ffplan[i], ffplan[i + 1]]))
                        i=i+2
                ffplan=ffPlan1
                print(ffplan)
                if ffplan:
                    self.mem.set(self.mem.META_PLAN, ffplan)
                    #print(self.mem.get(self.mem.META_PLAN))

                    if self.verbose >= 1: print(("    Selected plan: " + str(ffplan)))

    def plan_for_goal(self, goal):
        print(("-*-*- plan_for_goal(): goal = " + str(goal) + ", self.goals_to_plans = " + str(self.goals_to_plans)))
        plan = self.goals_to_plans[goal[0]]
        if goal[0] == "SWAP-MODULE":
            if self.mem.trace.module == "PyHopPlannerBroken":
                return self.ground_plan(plan, goal)
            else:
                # do a meaningless switch
                old_component = self.mem.trace.module
                new_component = self.mem.trace.module
                for operator in plan:
                    print(("operator is " + str(operator)))
                    for item in operator:
                        item.replace("?x", self.mem.trace.module)
        elif goal[0] == "ACHIEVEABLE-GOAL":
            return self.ground_plan(plan, goal, starting_goal=goal[1])
        else:
            raise Exception('UNDEFINED GOAL:', goal)

    # specific code to ground specific plans (temporary solution)
    def ground_plan(self, ungrounded_plan, goal, starting_goal=False):

        grounded_plan = []
        if self.using_baxter and self.mem.trace.module == "PyHopPlannerBroken" and goal[0] == "SWAP-MODULE":
            phase = "Plan"
            old_component = self.mem.trace.module
            new_component = "AsynchPyhopPlanner"  # TODO: for now this is hardcoded knowledge
            if len(ungrounded_plan) == 2:
                action1 = [ungrounded_plan[0][0], ungrounded_plan[0][1].replace("?x", old_component)]
                grounded_plan.append(action1)
                action2 = [ungrounded_plan[1][0], ungrounded_plan[1][1].replace("?p", phase),
                           ungrounded_plan[1][2].replace("?x", new_component)]
                grounded_plan.append(action2)
        elif self.mem.trace.module == "PyHopPlannerBroken" and goal[0] == "SWAP-MODULE":
            phase = "Plan"
            old_component = self.mem.trace.module
            new_component = "PyHopPlanner"  # TODO: for now this is hardcoded knowledge
            if len(ungrounded_plan) == 2:
                action1 = [ungrounded_plan[0][0], ungrounded_plan[0][1].replace("?x", old_component)]
                grounded_plan.append(action1)
                action2 = [ungrounded_plan[1][0], ungrounded_plan[1][1].replace("?p", phase),
                           ungrounded_plan[1][2].replace("?x", new_component)]
                grounded_plan.append(action2)
        elif "Planner" in self.mem.trace.module and goal[0] == "ACHIEVEABLE-GOAL":
            for operator in ungrounded_plan:
                grounded_operator = [starting_goal if x == "?g" else x for x in operator]
                grounded_plan.append(grounded_operator)
        else:
            raise Exception('No ground_plan protocol for:', self.mem.trace.module, goal)

        # if (self.verbose >= 2): print("-*-*- ground_plan(): returning "+str(grounded_plan))
        return grounded_plan
