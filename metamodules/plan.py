class MRSimplePlanner:

    goals_to_plans = None
    default_goals_to_plans = {"SWAP-MODULE":[["REMOVE-MODULE", "?x"],["ADD-MODULE","?p","?x"]]}
    trace = None

    def __init__(self, verbose = 0):
        self.goals_to_plans = self.default_goals_to_plans
        # TODO get trace from mem
        #self.trace = trace
        self.verbose = verbose

    def plan_for_goal(self, goal):
        #print("-*-*- plan_for_goal(): goal = "+str(goal)+", self.goals_to_plans = "+str(self.goals_to_plans))
        plan = self.goals_to_plans[goal[0]]
        if goal[0] == "SWAP-MODULE":
            if self.trace.module == "PyHopPlannerBroken":
                return self.ground_plan(plan, goal)
            else:
                # do a meaningless switch
                old_component = self.trace.module
                new_component = self.trace.module
                for operator in plan:
                    operator.replace("?x", self.trace.module)
        else:
            raise Exception('UNDEFINED GOAL:',goal)

    # specific code to ground specific plans (temporary solution)
    def ground_plan(self, ungrounded_plan, goal):
        grounded_plan = []
        if self.trace.module == "PyHopPlannerBroken" and goal[0] == "SWAP-MODULE":
            phase = "Plan"
            old_component = self.trace.module
            new_component = "PyHopPlanner" # TODO: for now this is hardcoded knowledge
            if len(ungrounded_plan) == 2:
                action1 = [ungrounded_plan[0][0], ungrounded_plan[0][1].replace("?x", old_component)]
                grounded_plan.append(action1)
                action2 = [ungrounded_plan[1][0], ungrounded_plan[1][1].replace("?p", phase),ungrounded_plan[1][2].replace("?x", new_component)]
                grounded_plan.append(action2)
        else:
            raise Exception('No ground_plan protocol for:',self.trace.module, goal)

        if (self.verbose >= 2): print("-*-*- ground_plan(): returning "+str(grounded_plan))
        return grounded_plan
