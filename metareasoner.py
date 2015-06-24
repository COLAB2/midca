

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

    def __init__(self, trace, mem):
        self.trace = trace
        self.cognitive_layer = mem.myMidca
        
        self.discrepancy_detector = MRSimpleDetect(self.trace)
        self.goal_formulator = MRSimpleGoalGen(self.trace)
        self.planner = MRSimplePlanner(self.trace)
        self.controller = MRSimpleControl(self.cognitive_layer)
        print("-*-*- MetaReasoner.__init__() successful")

    # goes through the metareasoning cycle
    def run(self):
        # interpret
        print("-*-*- MetaReasoner starting run")
        anomalies = self.discrepancy_detector.detect() 
        print("-*-*- MetaReasoner anomalies detected: "+str(anomalies))        
        new_goals = []
        for anom in anomalies:
            new_goals.append(self.goal_formulator.gen_goal(anom))

        print("-*-*- MetaReasoner goals are: "+str(new_goals))                
        # implicit intend: pursue all goals    

        # plan    
        plans = []    
        for new_goal in new_goals:
            plans.append(self.planner.plan_for_goal(new_goal))

        print("-*-*- MetaReasoner plans are: "+str(plans))                
        # controller
        for plan in plans:
            print("-*-*- MetaReasoner about to execute action: "+str(plan[0]))                            
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

    def detect(self):
        anomalies = []
        # only check last phase of trace
        prev_phase_data = self.trace.get_data(self.trace.get_current_cycle(), self.trace.get_current_phase())
        #print("-*-*- detect(): prev_phase_data = " + str(prev_phase_data))        
        #print("-*-*- detect(): self.neg_expectations.keys(): is " + str(self.neg_expectations.keys()))                                
        # see if any expectations exist for this phase
        if self.trace.get_current_phase() in self.neg_expectations.keys():            
            relevant_neq_exp = self.neg_expectations[self.trace.get_current_phase()]
            
            for prev_phase_datum in prev_phase_data:
                #print("-*-*- detect(): prev_phase_datum is " + str(prev_phase_datum))

                if prev_phase_datum[0] in relevant_neq_exp.keys():
                    #print("-*-*- detect(): prev_phase_datum[0]: "+str(prev_phase_datum[0])+" found in " + str(relevant_neq_exp))

                    exp_to_check = relevant_neq_exp[prev_phase_datum[0]]

                    for exp in exp_to_check:
                        if prev_phase_datum[1] == exp[0]:
                            #print("-*-*- detect(): adding anomaly: "+str(exp[1]))
                            anomalies.append(exp[1])                            
                    
                    # for data in prev_phase_data:
                    #     # check against expectations
                    #     if data[0] in self.neg_expectations[prev_phase_data].keys():
                    #         for exp in self.neg_expectations[prev_phase_data][data[0]]:
                    #             if data[1] == exp[0]:
                    #             # anomaly detected in negative expectations


        # TODO: implement pos_expectations
        print("-*-*- detect(): returning anomalies: "+str(anomalies))
        return anomalies
    
class MRSimpleGoalGen:
    
    anoms_to_goals = None
    default_anoms_to_goals = {"IMPASSE":["SWAP-COMPONENT","?phase"]}
    trace = None
    
    def __init__(self, trace):
        self.anoms_to_goals = self.default_anoms_to_goals
        self.trace = trace
        
    def gen_goal(self,anomaly):
        ungrounded_goal = self.anoms_to_goals[anomaly]
        grounded_goal = []
        for item in ungrounded_goal:
            if item == "?phase":
                item = self.trace.phase
            grounded_goal.append(item)
                
        return grounded_goal
        
class MRSimplePlanner:
    
    goals_to_plans = None
    default_goals_to_plans = {"SWAP-COMPONENT":[["REMOVE-COMPONENT", "?x"],["ADD-COMPONENT","?x"]]}
    trace = None
    
    def __init__(self, trace):
        self.goals_to_plans = self.default_goals_to_plans
        self.trace = trace
        
    def plan_for_goal(self, goal):
        #print("-*-*- plan_for_goal(): goal = "+str(goal)+", self.goals_to_plans = "+str(self.goals_to_plans))        
        plan = self.goals_to_plans[goal[0]]
        if goal[0] == "SWAP-COMPONENT":
            if self.trace.phase == "PyHopPlanner":
                return self.ground_plan(plan, goal)
            else:
                # do a meaningless switch
                old_component = self.trace.phase
                new_component = self.trace.phase
                for operator in plan:
                    operator.replace("?x", self.trace.phase)
        else:
            raise Exception('UNDEFINED GOAL:',goal)

    # specific code to ground specific plans (temporary solution)
    def ground_plan(self, ungrounded_plan, goal):
        grounded_plan = []
        if self.trace.phase == "PyHopPlanner" and goal[0] == "SWAP-COMPONENT":
            old_component = self.trace.phase
            new_component = "PyHopPlanner2" # TODO: for now this is hardcoded knowledge
            if len(ungrounded_plan) == 2:
                action1 = [ungrounded_plan[0][0], ungrounded_plan[0][1].replace("?x", old_component)]
                grounded_plan.append(action1)
                action2 = [ungrounded_plan[1][0], ungrounded_plan[1][1].replace("?x", new_component)]
                grounded_plan.append(action2)
        else:
            raise Exception('No ground_plan protocol for:',self.trace.phase, goal)

        print("-*-*- ground_plan(): returning "+str(grounded_plan))
        return grounded_plan
                
            
class MRSimpleControl:
    cognitive_layer = None
    def __init__(self, cognitive_layer):
        self.cognitive_layer = cognitive_layer

    def act(self, action):
        if action[0] == "REMOVE COMPONENT":
            # find the component
            module_index = -1
            phase = None
            for phase in self.cognitive_layer.get_phases:
                i = 0
                for mod in self.cognitive_layer.get_modules(phase):
                    if str(mod) == action[1]:
                        module_index = i
                        phase = phase
                        break
                    i += 1
            # remove the component
            if phase and module_index > -1:
                self.cognitive_layer.remove_module(phase, module_index)
        elif action[0] == "ADD COMPONENT":
            # add module into phase
            self.cognitive_layer.runtime_append_module
        
    