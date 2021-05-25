import socket
from ._xp_goal.parser import *
from ._xp_goal.traverser import *
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
                       print("Meta-AQUA goal generated:", apprehendGoal, end=' ')
                       if inserted:
                           print()
                       else:
                           print(". This goal was already in the graph.")
               else:
                   if verbose >= 2:
                       print("Meta-AQUA output unrecognized. No goal generated. Output:\n", end=' ')
                       text
        except socket.timeout:
            if verbose >= 1:
                print("Error: no data received from Meta-AQUA before timeout.")
        except:
            if verbose >= 1:
                print("Error reading from Meta-AQUA.", + str(text))
                try:
                    #print " Got:\n" + text
                    pass
                except NameError:
                    print(" Unable to read from socket.")


    def __del__(self):
        '''
        close socket on deletion.
        '''
        self.readS.shutdown(socket.SHUT_RDWR)
