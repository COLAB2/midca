from midca import goals, base

class SimpleNBeaconsExplain(base.BaseModule):
    '''
    Records the explanation when there is a discrepancy. Right now:
    EXPLANATION == None # means that no explanation is needed
    EXPLANATION == True # means we successfully generated an explanation
    EXPLANATION == False # means we were unable to generate an explanation for the given anomaly

    EXPLANATION_VAL = one of the possible values:
        'stuck'
        'wind'
    '''

    def init(self, world, mem):
        self.world = world
        self.mem = mem
        self.mem.set(self.mem.EXPLANATION, None)
        self.mem.set(self.mem.EXPLANATION_VAL, None)

    def run(self, cycle, verbose=2):
        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
        self.verbose = verbose
        explanation = None
        discrepancy = self.mem.get(self.mem.DISCREPANCY)

        if discrepancy:
            expected = discrepancy[0]
            actual = discrepancy[1]

            if len(expected) > 0 or len(actual) > 0:
                if self.verbose >= 1: print("Explaining discrepancy of expected "+str(list(map(str,expected)))+" vs. actual "+str(list(map(str,actual)))+"...")

                # first check to see if the agent got stuck
                for actual_atom in actual:
                    if 'stuck' in str(actual_atom):
                        if self.verbose >= 1: print("Explanation is that the agent is stuck")
                        explanation = True
                        self.mem.set(self.mem.EXPLANATION, explanation)
                        self.mem.set(self.mem.EXPLANATION_VAL, 'stuck')
                        if trace:
                            trace.add_data("EXPLANATION", explanation)
                        return

                # if not stuck, check to see in what direction the agent was blown and by how much
                expected_atom = expected[0]
                actual_atom = actual[0]

                expected_tile = str(expected_atom.args[1])
                actual_tile = str(actual_atom.args[1])

                exp_x = int(expected_tile[2:].split('y')[0])
                exp_y = int(expected_tile[2:].split('y')[1])
                act_x = int(actual_tile[2:].split('y')[0])
                act_y = int(actual_tile[2:].split('y')[1])

                explanation = False
                #print "Explanation is that the agent was blown "+str(exp_x-act_x)+" tile(s) to the west"
                if self.verbose >= 1: print("Could not generate an explanation")
                self.mem.set(self.mem.EXPLANATION, explanation)
                self.mem.set(self.mem.EXPLANATION_VAL, None)
                if trace:
                    trace.add_data("EXPLANATION", explanation)
                return
#
#                 if exp_x > act_x:
#                     # agent was blown east
#                     explanation = True
#                     print "Explanation is that the agent was blown "+str(exp_x-act_x)+" tile(s) to the west"
#                     self.mem.set(self.mem.EXPLANATION, True)
#                     self.mem.set(self.mem.EXPLANATION_VAL, 'wind west '+str(exp_x-act_x))
#                     if trace: trace.add_data("EXPLANATION", explanation)
#                     return
#                 elif exp_x < act_x:
#                     # agent was blown east
#                     explanation = True
#                     print "Explanation is that the agent was blown "+str(act_x-exp_x)+" tile(s) to the east"
#                     self.mem.set(self.mem.EXPLANATION, True)
#                     self.mem.set(self.mem.EXPLANATION_VAL, 'wind east '+str(act_x-exp_x))
#                     if trace: trace.add_data("EXPLANATION", explanation)
#                     return
#                 elif exp_y > act_y:
#                     # agent was blown south
#                     explanation = True
#                     print "Explanation is that the agent was blown "+str(exp_y-act_y)+" tile(s) to the north"
#                     self.mem.set(self.mem.EXPLANATION, True)
#                     self.mem.set(self.mem.EXPLANATION_VAL, 'wind north '+str(exp_y-act_y))
#                     if trace: trace.add_data("EXPLANATION", explanation)
#                     return
#                 elif act_y > exp_y:
#                     # agent was blown south
#                     explanation = True
#                     print "Explanation is that the agent was blown "+str(act_y-exp_y)+" tile(s) to the south"
#                     self.mem.set(self.mem.EXPLANATION, True)
#                     self.mem.set(self.mem.EXPLANATION_VAL, 'wind south '+str(act_y-exp_y))
#                     if trace: trace.add_data("EXPLANATION", explanation)
#                     return

        else:
            # no discrepancy, so no need to explain
            if self.verbose >= 1: print("No Discrepancy, not running explanation")

        return
