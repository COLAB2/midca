from MIDCA import base
import inspect 
from MIDCA.worldsim import domainread

# profiling
import cProfile, pstats, StringIO

class MentalExpectation:
    '''
    An expectation about a mental state (which is a collection of variable bindings
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
        Given a segment of length one of a trace, return a dict of the variable
        names and the results of calling the functions on the values of those variables
        
        returns True or False, True if all expectations, False if not
        '''
        all_true = True
        at_least_one_true = False
        for [var,val] in var_vals:
            if var in self.vars_and_funcs.keys():
                #print "var is "+str(var)
                #print "val is "+str(val)
                if self.vars_and_funcs[var](val):
                    at_least_one_true = True
                    #print "TRUE"
                else:
                    #print "FALSE"
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
        action needs to be a string which is the 
        '''
        self.verbose = verbose
        if (priorMentalExp is None and postMentalExp is None):
            raise Exception("Cannot create primitive expectaiton of cognition with no mental expectations")
        
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
            print "*^*^*^*^*^*^*^* WE HAVE AN EXPECTATION VIOLATION!!!!! *^*^*^*^*^*^*^*^*^*"
            return True
        else:
            if self.verbose >= 1: print "    ALL GOOD SIR"
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

    # negative expectations: if equal to observed state, anomaly detected
    #neg_expectations = {"PyHopPlannerBroken":
    #                    {"PLAN":[[None,"IMPASSE"]],
    #                     "INPUT":[]}}
    #pos_expectations = {}

    def init(self, world, mem):
        self.world = world
        self.mem = mem
        self.verbose = 0 # set at zero, change later during run()
        self.already_switched_moveeast = False
        self.wind_str = 0
        self.exp1 = self.createAlwaysExplanationExp()
         
    
    def createAlwaysExplanationExp(self):
        # creates the expectation that there is always an explanation following an 
        # discrepancy at the cognitive level
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
        return new_op_str
    
    def run(self, cycle, verbose=2):
        #pr = cProfile.Profile()
        #pr.enable()
        self.verbose = verbose
        
        #print str(exp1)
        
        # get the current data from the trace
        #post = self.mem.trace.get_n_prev_phase()
        # get the 2nd most
        #prior = self.mem.trace.get_n_prev_phase(n=1)
         
        exp_violation = self.exp1.apply_on_curr_trace(self.mem)
        
        if exp_violation:
            # hack, just go ahead and update the operator here
            # always assume move east, just update it assuming
            new_move_op_str = ""
            if self.wind_str == 0:
                self.wind_str +=1
                new_move_op_str = self.get_new_moveeast(self.wind_str)
            elif self.wind_str == 1:
                self.wind_str +=1
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
                print "Now creating the new operator"
                worldsim_op = domainread.load_operator_str(new_move_op_str)
                print "We now have worldsim op "+str(worldsim_op)
                print "Adding it into the world"
                self.world.operators[worldsim_op.name] = worldsim_op    
                print "Saving world into memory"    
                #self.mem.add(self.mem.STATES, self.world)
                self.already_switched_moveeast = True
                #print "Now adding in the new operator"
                
                if self.verbose >= 1: print "The following operators are available for planning: "
                if self.verbose >= 1: 
                    for op_k,op_v in self.world.operators.items():
                        print "    op["+str(op_k)+"] = "+str(op_v)
                
                
            except:
                pass
            
        
        
            #try:
        #    world = self.mem.get(self.mem.STATES)[-1]
            #print "The following operators are in the most recent world state: "
            #for op_k,op_v in world.operators.items():
            #    print "    op["+str(op_k)+"] = "+str(op_v)
        #except:
        #    pass    
            
    
        
#         print("prior is \n"+str(prior))
#         if prior:
#             print("prior phase is "+str(prior[0][1]))
#             print("prior variables are "+str(prior[1]))
#         
#         print("post is \n"+str(post))
#         if post:
#             print("post phase is "+str(post[0][1]))
#             print("post variables are "+str(post[1]))
#         
        
        # do matching
         
#         pr.disable()
#         s = StringIO.StringIO()
#         sortby = 'tottime'
#         ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
#         ps.print_stats()
#         print s.getvalue()
#         
        
        #self.detect()


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
