import socket
from MIDCA.modules._xp_goal.parser import * 
from MIDCA.modules._xp_goal.traverser import *
from MIDCA import goals, base

class MAQuery(base.BaseModule):

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
               frames = p.makeframegraph(text)
               noem = {}   # Node Operator Effect Mapping
               noem['CRIMINAL-VOLITIONAL-AGENT.4697'] = [['apprehend', OPERATOR_EFFECT_NEGATION]]
               t = Traverser(frames, noem)
               (frame, operator, effect) = t.traverse()
               if operator == "apprehend":
                   apprehendGoal = goals.Goal("Gui Montag", predicate = "free",
                                              negate = True)
                   inserted = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                   if verbose >= 2:
                       print "Meta-AQUA goal generated:", goal,
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
                print "Error reading from Meta-AQUA.",
                try:
                    print " Got:\n" + text
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
    '''
    
    def init(self, world, mem):
        self.world = world
        self.mem = mem
        
    def run(self, cycle, verbose=2):
        self.verbose = verbose
        explanation = None
        discrepancy = self.mem.get(self.mem.DISCREPANCY)
        if discrepancy:
                
            expected = discrepancy[0]
            actual = discrepancy[1]
            
            
            if len(expected) > 0 or len(actual) > 0:
                print "Explaining discrepancy of expected "+str(map(str,expected))+" vs. actual "+str(map(str,actual))+"..."
                for ex_atom,actual_atom in zip(map(str,expected),map(str,actual)):
                    if "Tx" in ex_atom and "Tx" in actual_atom:
                        explanation = False
                        print "... Failed to Explaining location discrepancy"
                        
                if explanation is None:
                    explanation = True
                
        else:
            # no discrepancy, so no need to explain
            print "No Discrepancy, not running explanation"
            
        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("EXPLANATION", explanation)
        
        return
    