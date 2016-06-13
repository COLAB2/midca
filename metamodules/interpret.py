from MIDCA import base

class MRSimpleDetect(base.BaseModule):
    '''
    Used to detect the PyHopPlannerBroken class has failed to produce a plan
    '''
    # negative expectations: if equal to observed state, anomaly detected
    neg_expectations = {"PyHopPlannerBroken":
                        {"PLAN":[[None,"IMPASSE"]],
                         "INPUT":[]}}
    pos_expectations = {}

    def run(self, cycle, verbose=2):
        self.verbose = verbose
        self.detect()


    def detect(self):
        last_phase_name, last_phase_data = self.mem.get(self.mem.TRACE_SEGMENT)
        anomalies = []

        # see if any expectations exist for this phase
        if last_phase_name in self.neg_expectations.keys():
            relevant_neq_exp = self.neg_expectations[last_phase_name]

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
        if (self.verbose >= 2): print("    Found anomalies: "+str(anomalies))
        self.mem.set(self.mem.META_ANOMALIES, anomalies)

class MRSimpleDetect2(base.BaseModule):
    '''
    Used to detect the PyHopPlanner class has failed to produce a plan
    '''
    
    # negative expectations: if equal to observed state, anomaly detected
    neg_expectations = {"PyHopPlanner":
                        {"PLAN":[[None,"IMPASSE"]],
                         "INPUT":[]}}
    pos_expectations = {}

    def run(self, cycle, verbose=2):
        self.verbose = verbose
        self.detect()

    def detect(self):
        last_phase_name, last_phase_data = self.mem.get(self.mem.TRACE_SEGMENT)
        anomalies = []

        # see if any expectations exist for this phase
        if last_phase_name in self.neg_expectations.keys():
            relevant_neq_exp = self.neg_expectations[last_phase_name]

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
        if (self.verbose >= 2): print("    Found anomalies: "+str(anomalies))
        self.mem.set(self.mem.META_ANOMALIES, anomalies)


class MRSimpleGoalGen(base.BaseModule):
    '''
    This class generates a goal to swap out the cognitive level plan module.
    '''
    anoms_to_goals = {"IMPASSE":["SWAP-MODULE","?phase"]}

    def run(self,cycle,verbose=2):
        self.verbose = verbose
        #print("[in MRSimpleGoalGen] self.mem.META_ANOMALIES are "+str(self.mem.get(self.mem.META_ANOMALIES)))
        if not self.mem.get(self.mem.META_GOALS):
            self.mem.set(self.mem.META_GOALS, [])

        if self.mem.get(self.mem.META_ANOMALIES):
            goals = self.mem.get(self.mem.META_GOALS)
            for anomaly in self.mem.get(self.mem.META_ANOMALIES):
                new_goal = self.gen_goal(anomaly)
                goals.append(new_goal)
                self.mem.set(self.mem.META_GOALS, goals)
                
        #print("mem.META_GOALS is now: "+str(self.mem.get(self.mem.META_GOALS)))
    def gen_goal(self, anomaly):
        ungrounded_goal = self.anoms_to_goals[anomaly]
        grounded_goal = []
        for item in ungrounded_goal:
            if item == "?phase":
                item = self.mem.trace.module
            grounded_goal.append(item)

        return grounded_goal

class MRSimpleGoalGenForGoalTrans(base.BaseModule):
    '''
    This class generates a goal to transform the goal when the planner fails.
    '''
    anoms_to_goals = {"IMPASSE":["ACHIEVEABLE-GOAL","?goal"]}

    def run(self,cycle,verbose=2):
        self.verbose = verbose
        #print("[in MRSimpleGoalGen] self.mem.META_ANOMALIES are "+str(self.mem.get(self.mem.META_ANOMALIES)))
        if not self.mem.get(self.mem.META_GOALS):
            self.mem.set(self.mem.META_GOALS, [])

        if self.mem.get(self.mem.META_ANOMALIES):
            goals = self.mem.get(self.mem.META_GOALS)
            for anomaly in self.mem.get(self.mem.META_ANOMALIES):
                new_goal = self.gen_goal(anomaly, verbose)
                goals.append(new_goal)
                self.mem.set(self.mem.META_GOALS, goals)
        #print("mem.META_GOALS is now: "+str(self.mem.get(self.mem.META_GOALS)))
        
    def gen_goal(self, anomaly, verbose=2):
        ungrounded_goal = self.anoms_to_goals[anomaly]
        grounded_goal = []
        for item in ungrounded_goal:
            if item == "?phase":
                item = self.mem.trace.module
            if item == "?goal":
                # get the most recent goal sent to planner from the trace
                if verbose >= 2: print("self.mem.trace.module = "+str(self.mem.trace))
                if verbose >= 2: print("self.mem.trace.get_current_phase_data() = "+str(self.mem.trace.get_current_phase_data()))
                goals = self.mem.trace.get_current_phase_data()[1][1:][0] # ugly, i know
                if verbose >= 2: print("goals are = "+str(map(str, goals)))
                item = goals
            grounded_goal.append(item)
        if verbose >= 2: print("grounded_goal = "+str(grounded_goal))
        return grounded_goal

