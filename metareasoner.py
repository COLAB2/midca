from modules import planning

"""Author: Dustin Dannenhauer (dustin.td@gmail.com)

This class represents an initial metacognitive layer for MIDCA to be
run after each phase of the cognitive level.

"""

class MetaReasoner:
    trace = None
    discrepancy_detector = None
    planner = None
    controller = None
    cognitive_layer = None

    def __init__(self, trace, mem, verbose = 0):
        """ Verbose value of 1 is top level output, verbose 2 is more detailed """
        self.trace = trace
        self.cognitive_layer = mem.myMidca
        self.verbose = verbose
        self.monitor = MRSimpleMonitor(self.trace, verbose)
        self.discrepancy_detector = MRSimpleDetect(self.trace, verbose)
        self.goal_formulator = MRSimpleGoalGen(self.trace, verbose)
        self.planner = MRSimplePlanner(self.trace, verbose)
        self.controller = MRSimpleControl(self.cognitive_layer, verbose)
        if (self.verbose >= 1): print("-*-*- MetaReasoner initialized")

    # goes through the metareasoning cycle
    def run(self):
        # interpret
        if (self.verbose >= 1): print("-*-*- MetaReasoner starting...")
        last_phase_from_trace = self.monitor.get_last_phase()
        if (self.verbose >= 1): print("-*-*- MetaReasoner monitor retrieving trace (last phase only)")
        anomalies = self.discrepancy_detector.detect(last_phase_from_trace)
        if (self.verbose >= 1): print("-*-*- MetaReasoner anomalies detected: "+str(anomalies))
        new_goals = []
        for anom in anomalies:
            new_goals.append(self.goal_formulator.gen_goal(anom))

        if (self.verbose >= 1): print("-*-*- MetaReasoner goals are: "+str(new_goals))
        # implicit intend: pursue all goals

        # plan
        plans = []
        for new_goal in new_goals:
            plans.append(self.planner.plan_for_goal(new_goal))

        if (self.verbose >= 1): print("-*-*- MetaReasoner plans are: "+str(plans))
        # controller
        for plan in plans:
            for action in plan:
                if (self.verbose >= 1): print("-*-*- MetaReasoner about to execute action: "+str(action))
                self.controller.act(action)

class MRSimpleMonitor:

    def __init__(self,trace,verbose):
        self.trace = trace
        self.verbose = verbose

    def get_last_phase(self):
        """ Return a small part of the trace """
        return self.trace.get_data(self.trace.get_current_cycle(), self.trace.get_current_phase())


class MRSimpleDetect:
    trace = None
    # negative expectations: if equal to observed state, anomaly detected
    neg_expectations = {"PyHopPlannerBroken":
                        {"PLAN":[[None,"IMPASSE"]],
                         "INPUT":[]}}
    pos_expectations = {}

    def __init__(self, trace, verbose = 0):
        self.trace = trace
        self.verbose = verbose

    def detect(self, last_phase_data):
        anomalies = []

        # see if any expectations exist for this phase
        if self.trace.get_current_phase() in self.neg_expectations.keys():
            relevant_neq_exp = self.neg_expectations[self.trace.get_current_phase()]

            for prev_phase_datum in last_phase_data:
                #print("-*-*- detect(): prev_phase_datum is " + str(prev_phase_datum))

                if prev_phase_datum[0] in relevant_neq_exp.keys():
                    #print("-*-*- detect(): prev_phase_datum[0]: "+str(prev_phase_datum[0])+" found in " + str(relevant_neq_exp))

                    exp_to_check = relevant_neq_exp[prev_phase_datum[0]]

                    for exp in exp_to_check:
                        if prev_phase_datum[1] == exp[0]:
                            #print("-*-*- detect(): adding anomaly: "+str(exp[1]))
                            anomalies.append(exp[1])

                    # for data in last_phase_data:
                    #     # check against expectations
                    #     if data[0] in self.neg_expectations[last_phase_data].keys():
                    #         for exp in self.neg_expectations[last_phase_data][data[0]]:
                    #             if data[1] == exp[0]:
                    #             # anomaly detected in negative expectations


        # TODO: implement pos_expectations
        if (self.verbose >= 2): print("-*-*- detect(): returning anomalies: "+str(anomalies))
        return anomalies

class MRSimpleGoalGen:

    anoms_to_goals = None
    default_anoms_to_goals = {"IMPASSE":["SWAP-MODULE","?phase"]}
    trace = None

    def __init__(self, trace, verbose = 0):
        self.anoms_to_goals = self.default_anoms_to_goals
        self.trace = trace
        self.verbose = verbose

    def gen_goal(self, anomaly):
        ungrounded_goal = self.anoms_to_goals[anomaly]
        grounded_goal = []
        for item in ungrounded_goal:
            if item == "?phase":
                item = self.trace.module
            grounded_goal.append(item)

        return grounded_goal

class MRSimplePlanner:

    goals_to_plans = None
    default_goals_to_plans = {"SWAP-MODULE":[["REMOVE-MODULE", "?x"],["ADD-MODULE","?p","?x"]]}
    trace = None

    def __init__(self, trace, verbose = 0):
        self.goals_to_plans = self.default_goals_to_plans
        self.trace = trace
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


class MRSimpleControl:
    cognitive_layer = None
    def __init__(self, cognitive_layer, verbose = 0):
        self.cognitive_layer = cognitive_layer
        self.verbose = verbose

    def act(self, action, verbose = 0):
        if action[0] == "REMOVE-MODULE":
            # find the component
            module_index = -1
            phase = None
            mod_str = ""
            class Found(Exception): pass # is this bad python? should this go at top of my file?
            try:
                for phasei in self.cognitive_layer.get_phases():
                    i = 0
                    for mod in self.cognitive_layer.get_modules(phasei):
                        mod_str = str(mod.__class__.__name__)
                        #print("-*-*- act():  mod = "+mod_str+", action[1] = "+str(action[1]))
                        if mod_str == action[1]:
                            #print("-*-*- act(): we got a match!")
                            module_index = i
                            phase = phasei
                            raise Found
                        i += 1
            except Found:

                # remove the component
                #print("-*-*- act():  phase = "+str(phase)+", module_index = "+str(module_index))
                if phase and module_index > -1:
                    self.cognitive_layer.remove_module(phase, module_index)
                    is_success = mod_str not in map(lambda x: x.__class__.__name__, self.cognitive_layer.get_modules(phase))
                    if is_success: print("Metareasoner removed "+mod_str) # report any actions metareasoner carried out
                    if (self.verbose >= 2): print("-*-*- act():  did I succeed in removing PyHopPlannerBroken " + str(action[1])+": "+str(is_success)+" ")
        elif action[0] == "ADD-MODULE":
            if action[2] == "PyHopPlanner":
                self.cognitive_layer.runtime_append_module("Plan", planning.PyHopPlanner(True)) # TODO: hardcoded knowledge of Plan phase
                is_success = "PyHopPlanner" in map(lambda x: x.__class__.__name__, self.cognitive_layer.get_modules("Plan"))
                if (self.verbose >= 2): print("-*-*- act():  did I succeed in adding PyHopPlanner? "+str(is_success))
                if is_success: print("Metareasoner added PyHopPlanner") # report any actions metareasoner carried out







