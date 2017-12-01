import socket
from midca.modules._xp_goal.parser import *
from midca.modules._xp_goal.traverser import *
from midca import goals, base
import socket

class MAQuery(base.BaseModule):
    '''
    Reads the ouput from Meta-Aqua, builds the meta-aqua frame system.
    Every frame has values,relations and roles.
    searches for CRIMINAL-VOLITIONAL-AGENT from the frames and
    adds a goal if the operator is apprehend.
    '''

    endMsg = "Done"
    readSize = 100000
    
    def __init__(self, readPort, waitTime = 5.0):       
        self.readS = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.readS.connect(("localhost", readPort))
        self.readS.settimeout(waitTime)

    def run(self, cycle, verbose = 2):
        try:
            text = self.readS.recv(self.readSize)
            if text != "None\n":
               p = Parser()
               # create frames
               frames = p.makeframegraph(text)
               noem = {}   # Node Operator Effect Mapping
               noem['CRIMINAL-VOLITIONAL-AGENT'] = [['apprehend', OPERATOR_EFFECT_NEGATION]]
               # traverser class, initializes frames and noem
               t = Traverser(frames, noem)
               # gets the frame operator and effect
               (frame, operator, effect) = t.traverse()
               if operator == "apprehend":
                   apprehendGoal = goals.Goal("Gui Montag", predicate = "free",
                                              negate = True)
                   inserted = self.mem.get(self.mem.GOAL_GRAPH).insert(apprehendGoal)
                   if verbose >= 2:
                       print "Meta-AQUA goal generated:", apprehendGoal,
                       if inserted:
                           print
                       else:
                           print ". This goal was already in the graph."
               else:
                   if verbose >= 2:
                       print "Meta-AQUA output unrecognized. No goal generated. Output:\n",
                       text
        except socket.timeout:
            if verbose >= 1:
                print "Error: no data received from Meta-AQUA before timeout."
        except:
            if verbose >= 1:
                print "Error reading from Meta-AQUA.", + str(text)
                try:
                    #print " Got:\n" + text
                    pass
                except NameError:
                    print " Unable to read from socket."
               

    def __del__(self):
        '''
        close socket on deletion. 
        '''
        self.readS.shutdown(socket.SHUT_RDWR)



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
                if self.verbose >= 1: print "Explaining discrepancy of expected "+str(map(str,expected))+" vs. actual "+str(map(str,actual))+"..."
                
                # first check to see if the agent got stuck
                for actual_atom in actual:
                    if 'stuck' in str(actual_atom):
                        if self.verbose >= 1: print "Explanation is that the agent is stuck"
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
                if self.verbose >= 1: print "Could not generate an explanation"
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
            if self.verbose >= 1: print "No Discrepancy, not running explanation"
        
        return
    