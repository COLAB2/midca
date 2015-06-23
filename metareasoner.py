

"""This class represents the initial metacognitive layer for MIDCA"""

class MetaReasoner:
    trace = None
    discrepancy_detector = None
    planner = None
    controller = None

    def __init__(self, trace):
        self.trace = trace
        self.discrepancy_detector = MRSimpleDetect()
        self.goal_formulator = MRSimpleGoalGen()
        self.planner = MRSimplePlanner()
        self.controller = MRSimpleControl()


    # goes through the metareasoning cycle
    def run(self):
        # interpret
        anomalies = self.discrepancy_detector.detect(self.trace)
        new_goals = []
        for anom in anomalies:
            new_goals.append(self.goal_formulator.gen_goal(anom))
            
        # implicit intend: pursue all goals    

        # plan    
        plans = []    
        for new_goal in new_goals:
            plans.append(self.planner.plan_for_goal(new_goal))
            
        # controller
        for plan in plans:
            self.controller.act(plan[0])
            
        
class MRSimpleDetect:
    trace = None
    # negative expectations: if equal to observed state, anomaly detected
    neg_expectations = {"PyHopPlanner":
                        {"PLAN":[[None,"IMPASSE"]],
                         "INPUT":[]}}
    pos_expectations = {}

    def __init__(self, trace):
        self.trace = trace

    def detect():
        anomalies = []
        # only check last phase of trace
        prev_phase = self.trace[self.trace.cycle][self.trace.phase]

        if prev_phase in neg_expectations.keys():

            for data in prev_phase:
                # check against expectations
                if data[0] in neg_expectations[prev_phase].keys():
                    for exp in neg_expectations[prev_phase][data[0]]:
                        if data[1] == exp[0]:
                            # anomaly detected in negative expectations
                            anomalies.append(exp[1])

        # TODO: implement pos_expectations
        return anomalies
    
class MRSimpleGoalGen:
    anoms_to_goals = None
    default_anoms_to_goals = {"IMPASSE":"SWAP-COMPONENT"}
    
    def __init__(self):
        self.anoms_to_goals = default_anoms_to_goals
        
    def gen_goal(anomaly):
        return self.anoms_to_goals[anomaly]
        

class MRSimplePlanner:
    goals_to_plans = None
    default_goals_to_plans = {"SWAP-COMPONENT":["REMOVE-COMPONENT ?x","ADD-COMPONENT ?x"]}
    trace = None
    
    def __init__(self, trace):
        self.goals_to_plans = default_goals_to_plans
        
    def plan_for_goal(goal):
        plan = self.goals_to_plans[goal]
        if goal == "SWAP-COMPONENT":
            for operator in plan:
                operator.replace("?x",)
    