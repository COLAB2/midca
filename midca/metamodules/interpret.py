from midca import base
import inspect 
from midca.worldsim import domainread

class MentalExpectation:
    '''
    An expectation about a mental state (a mental state is a collection of variable bindings)
    '''
    
    def __init__(self):
        self.vars_and_funcs = {}
        
    def add_var_expectaiton(self, varname, func):
        '''
        Adds an expectation about a single variable
        
        Example:
        varname = 'CURRENT GOALS'
        func = lambda x: len(x) > 0
        '''
    
        self.vars_and_funcs[varname] = func
    
    def get_vars_and_funcs(self):
        return self.vars_and_funcs
    
    def apply_expectation(self,var_vals):
        '''
        Given a segment of length 1 of a trace, return a dict of the variable
        names and the results of calling the functions on the values of those variables
        
        returns True or False, True if all expectations met, False if not
        '''
        all_true = True
        at_least_one_true = False
        for [var,val] in var_vals:
            if var in self.vars_and_funcs.keys():
                if self.vars_and_funcs[var](val):
                    at_least_one_true = True
                else:
                    all_true = False
        
        if all_true and at_least_one_true:
            #print "========>>>>>>> EXPECTATIONS HAVE BEEN MET"
            return True
        elif at_least_one_true:
            #print "========>>>>>>> AT LEAST ONE, BUT NOT ALL, OF THE EXPECTATIONS WERE MET"
            pass
        else:
            #print "========>>>>>>> NONE OF THE EXPECTATIONS WERE MET"
            pass
    
    
        return False
        #print "trace segment is "+str(trace_seg)

class PrimitiveExpectationOfCognition():
    '''
    An expectation of cognition involving one mental action (a single cognitive process)
    '''
    
    def __init__(self, action, priorMentalExp=None,postMentalExp=None, verbose=2):
        '''
        action refers to the mental action, and should be the string name of a module, like
        'PyHopPlanner' or 'SimpleNBeaconsExplain'
        '''
        self.verbose = verbose
        if (priorMentalExp is None and postMentalExp is None):
            raise Exception("Cannot create primitive expectation of cognition with no mental expectations")
        
        self.action = action
        self.priorMentalExp = priorMentalExp
        self.postMentalExp = postMentalExp
    
    def apply_on_curr_trace(self, mem):
        '''
        Returns true if expectation met, and false otherwise using the current trace stored in memory
        '''     
        trace = mem.trace
        
        # get the current data from the trace
        post = trace.get_n_prev_phase()
        # get the 2nd most
        prior = trace.get_n_prev_phase(n=1)
        
        if prior is None or post is None:
            return False 
        
        prior_result = self.priorMentalExp.apply_expectation(prior[1])
        post_result = self.postMentalExp.apply_expectation(post[1])
        
        #print "    Prior result is "+str(prior_result)
        #print "    Post result is "+str(post_result)
        
        if prior_result and not post_result:
            # print "*^*^*^*^*^*^*^* WE HAVE AN EXPECTATION VIOLATION!!!!! *^*^*^*^*^*^*^*^*^*"
            return True
        else:
            # if self.verbose >= 1: print "    ALL GOOD SIR: Expectations have been met"
            return False
         
    def __str__(self):
        s = 'Prim. Exp. of Cog. for '+str(self.action)+":\n"
        s += '  Prior Mental State Exp. :\n'
        for var,func in self.priorMentalExp.get_vars_and_funcs().items():
            s += '    '+str(var).strip()+": "+ str(inspect.getsource(func)).strip()+"\n"
        s += '  Post Mental State Exp. :\n'
        for var,func in self.postMentalExp.get_vars_and_funcs().items():
            s += '    '+str(var).strip()+": "+ str(inspect.getsource(func)).strip()+"\n"
        return s

class MRSimpleDetect(base.BaseModule):

    def init(self, world, mem):
        self.world = world
        self.mem = mem
        self.verbose = 0 # set at zero, change later during run()
        self.already_switched_moveeast = False
        self.wind_str = 0
        self.exp1 = self.createAlwaysExplanationExp()
        self.updated_to_move2 = False
        self.updated_to_move3 = False #hacky i know
        self.updated_to_move4 = False #hacky i know
    
    def createAlwaysExplanationExp(self):
        ''' 
        creates the expectation that there is always an explanation following a 
        discrepancy at the cognitive level
        '''
        
        priorExpectation = MentalExpectation()
        #discrepancyExists = lambda d: not (d is None) or (len(d[0]) > 0 or len(d[1]) > 0)
        discrepancyExists = lambda d: d # identity for now
        priorExpectation.add_var_expectaiton('DISCREPANCY', discrepancyExists)
        
        postExpectation = MentalExpectation()
        hasExplanation = lambda e: e # identity
        postExpectation.add_var_expectaiton('EXPLANATION', hasExplanation)
    
        # encode action
        action = 'SimpleNBeaconsExplain'
        
        return PrimitiveExpectationOfCognition(action, priorExpectation, postExpectation,verbose=self.verbose)
        
    def get_new_moveeast(self,wind_str):
        '''
        Given the strength of the wind, this function will return the new operator that will take
        wind into account. 
        '''
        
        mud = True
        new_op_str = ""
        if wind_str == 1:
            if mud:
                new_op_str = 'operator_no_side_effect(moveeast2, \
                        args = [(agnt, AGENT), (start, TILE), (mid, TILE), (dest, TILE)], \
                        preconditions = [ \
                            condition(free, [agnt]), \
                            condition(quicksand, [mid], negate = TRUE), \
                            condition(quicksand, [dest], negate = TRUE), \
                            condition(agent-at, [agnt, start]), \
                            condition(adjacent-east, [start, mid]), \
                            condition(adjacent-east, [mid, dest])], \
                        results = [ \
                            condition(agent-at, [agnt, start], negate = TRUE), \
                            condition(agent-at, [agnt, dest])])'
            else:
                new_op_str = 'operator_no_side_effect(moveeast2, \
                        args = [(agnt, AGENT), (start, TILE), (mid, TILE), (dest, TILE)], \
                        preconditions = [ \
                            condition(free, [agnt]), \
                            condition(agent-at, [agnt, start]), \
                            condition(adjacent-east, [start, mid]), \
                            condition(adjacent-east, [mid, dest])], \
                        results = [ \
                            condition(agent-at, [agnt, start], negate = TRUE), \
                            condition(agent-at, [agnt, dest])])'
        elif wind_str == 2:
            if mud:
                new_op_str = 'operator_no_side_effect(moveeast3, \
                        args = [(agnt, AGENT), (start, TILE), (mid, TILE), (mid2, TILE), (dest, TILE)], \
                        preconditions = [ \
                            condition(free, [agnt]), \
                            condition(quicksand, [mid], negate = TRUE), \
                            condition(quicksand, [mid2], negate = TRUE), \
                            condition(quicksand, [dest], negate = TRUE), \
                            condition(agent-at, [agnt, start]), \
                            condition(adjacent-east, [start, mid]), \
                            condition(adjacent-east, [mid, mid2]), \
                            condition(adjacent-east, [mid2, dest])], \
                        results = [ \
                            condition(agent-at, [agnt, start], negate = TRUE), \
                            condition(agent-at, [agnt, dest])])'
            else:
                new_op_str = 'operator_no_side_effect(moveeast3, \
                        args = [(agnt, AGENT), (start, TILE), (mid, TILE), (mid2, TILE), (dest, TILE)], \
                        preconditions = [ \
                            condition(free, [agnt]), \
                            condition(agent-at, [agnt, start]), \
                            condition(adjacent-east, [start, mid]), \
                            condition(adjacent-east, [mid, mid2]), \
                            condition(adjacent-east, [mid2, dest])], \
                        results = [ \
                            condition(agent-at, [agnt, start], negate = TRUE), \
                            condition(agent-at, [agnt, dest])])'
                            
        elif wind_str == 3:
            print "wind_str = "+str(wind_str)
            if mud:
                new_op_str = 'operator_no_side_effect(moveeast4, \
                        args = [(agnt, AGENT), (start, TILE), (mid, TILE), (mid2, TILE), (mid3, TILE), (dest, TILE)], \
                        preconditions = [ \
                            condition(free, [agnt]), \
                            condition(quicksand, [mid], negate = TRUE), \
                            condition(quicksand, [mid2], negate = TRUE), \
                            condition(quicksand, [mid3], negate = TRUE), \
                            condition(quicksand, [dest], negate = TRUE), \
                            condition(agent-at, [agnt, start]), \
                            condition(adjacent-east, [start, mid]), \
                            condition(adjacent-east, [mid, mid2]), \
                            condition(adjacent-east, [mid2, mid3]), \
                            condition(adjacent-east, [mid3, dest])], \
                        results = [ \
                            condition(agent-at, [agnt, start], negate = TRUE), \
                            condition(agent-at, [agnt, dest])])'
                print "new op str is now moveast4"
            else:
                raise Exception("SHOULD NOT BE HERE - move east 4 with no mud")
                new_op_str = 'operator_no_side_effect(moveeast3, \
                        args = [(agnt, AGENT), (start, TILE), (mid, TILE), (mid2, TILE), (dest, TILE)], \
                        preconditions = [ \
                            condition(free, [agnt]), \
                            condition(agent-at, [agnt, start]), \
                            condition(adjacent-east, [start, mid]), \
                            condition(adjacent-east, [mid, mid2]), \
                            condition(adjacent-east, [mid2, dest])], \
                        results = [ \
                            condition(agent-at, [agnt, start], negate = TRUE), \
                            condition(agent-at, [agnt, dest])])'
    
    
        elif wind_str == 4:
            print "wind_str = "+str(wind_str)
            if mud:
                new_op_str = 'operator_no_side_effect(moveeast5, \
                        args = [(agnt, AGENT), (start, TILE), (mid, TILE), (mid2, TILE), (mid3, TILE), (mid4, TILE), (dest, TILE)], \
                        preconditions = [ \
                            condition(free, [agnt]), \
                            condition(quicksand, [mid], negate = TRUE), \
                            condition(quicksand, [mid2], negate = TRUE), \
                            condition(quicksand, [mid3], negate = TRUE), \
                            condition(quicksand, [mid4], negate = TRUE), \
                            condition(quicksand, [dest], negate = TRUE), \
                            condition(agent-at, [agnt, start]), \
                            condition(adjacent-east, [start, mid]), \
                            condition(adjacent-east, [mid, mid2]), \
                            condition(adjacent-east, [mid2, mid3]), \
                            condition(adjacent-east, [mid3, mid4]), \
                            condition(adjacent-east, [mid4, dest])], \
                        results = [ \
                            condition(agent-at, [agnt, start], negate = TRUE), \
                            condition(agent-at, [agnt, dest])])'
                print "new op str is now moveast5"
    
            else:
                raise Exception("WHY IS MUD FALSE?????? ")
                # TODO: some old code could lead to here if mud is false, need to update
            
            
        return new_op_str
    
    def run(self, cycle, verbose=2):
        self.verbose = verbose
        
        #print str(exp1)
        
        # get the current data from the trace
        #post = self.mem.trace.get_n_prev_phase()
        # get the 2nd most
        #prior = self.mem.trace.get_n_prev_phase(n=1)
         
        exp_violation = self.exp1.apply_on_curr_trace(self.mem)
        
        if exp_violation:
            
            # get the distance the wind pushed the agent from its start location
            # agents last action:
            last_actions = None
            dist = -1
            try:
                last_actions = self.mem.get(self.mem.ACTIONS)[-1]
                last_action = last_actions[0]
                last_action_start = str(last_action.args[1])
                #print "last action start is "+str(last_action_start)
                agents_loc = str(self.world.get_atoms(filters="agent-at")[0].args[1])
                #print "agents_loc is "+str(agents_loc)
                # now compute distance because its relevant
                last_action_start = last_action_start[2:] #remove 'Tx'
                start_x = int(last_action_start.split('y')[0])
                start_y = int(last_action_start.split('y')[1])
                #print "here"
                agent_loc = agents_loc[2:] # remove the 'Tx'
                agent_x = int(agent_loc.split('y')[0])
                agent_y = int(agent_loc.split('y')[1])
                #print "here"
                dist = (abs(start_x - agent_x)+abs(start_y-agent_y))
                self.wind_str = dist-1
                print "Recorded a wind strength of "+str(self.wind_str)
                
            except:
                raise Exception("Problem getting last action in meta interpret")
            
            # hack, just go ahead and update the operator here
            # always assume move east, just update it assuming
            new_move_op_str = ""
            new_move_op_str = self.get_new_moveeast(self.wind_str)
                
            try:
                # 1. get the current world state
                #world = self.mem.get(self.mem.STATES)[-1]
                
                # 2. find the old operator moveeast
                prev_move_opname = ''
                for op in self.world.get_operators().values():
                    if 'moveeast' in op.name:
                        if self.verbose >= 1: print "Found operator moveeast: \n"
                        if self.verbose >= 1: print "  "+str(op)
                        prev_move_opname = op.name
                
                # 3. Remove the old move operator
                rem_succes = self.world.remove_operator(prev_move_opname)
                
                print "The removal of operator "+str(prev_move_opname)+" was successful: "+str(rem_succes)
                    
                    
                # 4. replace with new moveeast operator
                #print "Now creating the new operator"
                #print "new op str is now: "
                #print str(new_move_op_str)
                worldsim_op = domainread.load_operator_str(new_move_op_str)
                #print "We now have worldsim op "+str(worldsim_op)
                #print "Adding it into the world"
                self.world.operators[worldsim_op.name] = worldsim_op    
                #print "Saving world into memory"    
                #self.mem.add(self.mem.STATES, self.world)
                self.already_switched_moveeast = True
                print "Successfully added in the new operator "+str(worldsim_op.name)
                
                if self.verbose >= 1: print "The following operators are available for planning: "
                if self.verbose >= 1: 
                    for op_k,op_v in self.world.operators.items():
                        print "    op["+str(op_k)+"] = "+str(op_v)
    
            except:
                pass


    
class MRSimpleDetectOld(base.BaseModule):
    '''
    Used to detect the PyHopPlanner class has failed to produce a plan
    '''
    
    # negative expectations: if equal to observed state, anomaly detected
    neg_expectations = {"PyHopPlannerBroken":
                        {"PLAN":[[None,"IMPASSE"]],
                         "INPUT":[]}}
    pos_expectations = {}

    def init(self, world, mem):
        self.world = world
        self.mem = mem

    def run(self, cycle, verbose=2):
        self.verbose = verbose
        self.detect()

    def detect(self):
        last_phase_name, last_phase_data = self.mem.get(self.mem.TRACE_SEGMENT)
        anomalies = []
        
        # see if any expectations exist for this phase
        if last_phase_name in self.neg_expectations.keys():
            relevant_neg_exp = self.neg_expectations[last_phase_name]

            for prev_phase_datum in last_phase_data:
                print("-*-*- detect(): prev_phase_datum is " + str(prev_phase_datum))

                if prev_phase_datum[0] in relevant_neg_exp.keys():
                    print("-*-*- detect(): prev_phase_datum[0]: "+str(prev_phase_datum[0])+" found in " + str(relevant_neg_exp))

                    exp_to_check = relevant_neg_exp[prev_phase_datum[0]]

                    for exp in exp_to_check:
                        if prev_phase_datum[1] == exp[0]:
                            print("-*-*- detect(): adding anomaly: "+str(exp[1]))
                            anomalies.append(exp[1])

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

class MRSimpleDetect_construction(base.BaseModule):
    '''
    Used to detect the PyHopPlanner class has failed to produce a plan
    '''
    
    # negative expectations: if equal to observed state, anomaly detected
    neg_expectations = {"PyHopPlanner_construction":
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




