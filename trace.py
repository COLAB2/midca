from __future__ import print_function
from MIDCA.mem import Memory
from MIDCA.modules import planning2

class CogTrace:
    # trace[<cycle>][<module-id>] returns a list of what happened in
    # that module in that cycle
    trace = {}
    
    def __init__(self, mem):
        print("Calling init! Trace is: "+str(self.trace))
        self.mem = mem

    # def getInstance():
    #     if instance == None:
    #         return CogTrace()
    #     else:
    #         return instance
        
    def addphase(self, cycle, phase, data):
        """args: 
           cycle := integer representing a cycle
           phase := string representing the name of a phase """
    
        if cycle in self.trace.keys():
            if len(self.trace[cycle]) > 0 and phase in self.trace[cycle].keys():
                #print("Changing data for phase "+str(phase)+" in cycle "+str(cycle)+ " to " + str(data))
                self.trace[cycle][phase] = data
            else:
                #print("Fresh insert of data for phase "+str(phase)+" in cycle "+str(cycle)+ " to " + str(data))
                self.trace[cycle][phase] = data            
        else:
            self.trace[cycle] = {}
            #print("Fresh insert of data for phase "+str(phase)+" in cycle "+str(cycle)+ " to " + str(data))
            self.trace[cycle][phase] = data                    


    # When this is called, it means the current phase failed for some reason
    def failuredetected(self):
        # get the phase
        #failed_phase = self.trace[-1].keys()[0]
        #print("[trace.py] failed_phase is " + str(failed_phase))
        # if it's a planning phase then switch domain files
        #if failed_phase == "PyHopPlanner":
        self.mem.myMidca.clear_phase("Plan")
        self.mem.myMidca.append_module("Plan", planning2.PyHopPlanner(True))
        print("swapped out the planner, try again")            
            
    def printtrace(self):
        for cycle in self.trace.keys():
            for phase in self.trace[cycle].keys():
                print("---------------------------------------------\n[cycle  "+str(cycle)+"][phase "+str(phase)+"]\n---------------------------------------------\n\n"+str(self.trace[cycle][phase]))




