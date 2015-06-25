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


