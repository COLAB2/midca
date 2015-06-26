from metamodules import monitor, interpret, plan, control

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
        self.monitor = monitor.MRSimpleMonitor(self.trace, verbose)
        self.discrepancy_detector = interpret.MRSimpleDetect(self.trace, verbose)
        self.goal_formulator = interpret.MRSimpleGoalGen(self.trace, verbose)
        self.planner = plan.MRSimplePlanner(self.trace, verbose)
        self.controller = control.MRSimpleControl(self.cognitive_layer, verbose)
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












