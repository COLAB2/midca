from __future__ import print_function
from MIDCA.mem import Memory
from MIDCA.modules import planning2

"""
How-To: 
1. In each module, before storing data into the trace call add_phase() then
2. for each piece of data you want to add into the trace, call add_data()
"""

class CogTrace:
    # trace[<cycle>][<module-id>] returns a list of what happened in
    # that module in that cycle
    trace = {}
    cycle = -1 # current cycle
    phase = "" # current phase

    
    
    def __init__(self, mem):
        print("Calling init! Trace is: "+str(self.trace))
        self.mem = mem

    # def getInstance():
    #     if instance == None:
    #         return CogTrace()
    #     else:
    #         return instance
        
    def add_phase(self, cycle, phase):
        """args: 
           cycle := integer representing a cycle
           phase := string representing the name of a phase """
    
        if cycle in self.trace.keys():
            if len(self.trace[cycle]) > 0 and phase in self.trace[cycle].keys():
                #print("Changing data for phase "+str(phase)+" in cycle "+str(cycle)+ " to " + str(data))
                self.trace[cycle][phase] = []
            else:
                #print("Fresh insert of data for phase "+str(phase)+" in cycle "+str(cycle)+ " to " + str(data))
                self.trace[cycle][phase] = []            
        else:
            self.trace[cycle] = {}
            #print("Fresh insert of data for phase "+str(phase)+" in cycle "+str(cycle)+ " to " + str(data))
            self.trace[cycle][phase] = []
            
        self.cycle = cycle
        self.phase = phase

    def add_data(self, data_type, data):
        """
        data_type is one of "WORLD, GOALS, etc"
        """
        if self.cycle != -1 and self.phase != "":
            self.trace[self.cycle][self.phase].append([data_type,data])
            
            
    # When this is called, it means the current phase failed for some reason
    def failuredetected(self):
        # get the phase
        #failed_phase = self.trace[-1].keys()[0]
        #print("[trace.py] failed_phase is " + str(failed_phase))
        # if it's a planning phase then switch domain files
        #if failed_phase == "PyHopPlanner":
        self.mem.myMidca.clear_phase("Plan")
        self.mem.myMidca.runtime_append_module("Plan", planning2.PyHopPlanner(True))
        print("swapped out the planner, try again")            


    def print_data(self, data_type, data):
        if data_type == "WORLD":
            print("  WORLD: "+str(data))
        elif data_type == "PREV WORLD":
            print("  PREV WORLD: "+str(data))
        elif data_type == "CURR WORLD":
            print("  CURR WORLD: "+str(data))            
        elif data_type == "GOALS":
            if data is None:
                print("  GOALS: []")
            else:
                print("  GOALS: ")                
                for g in data:
                    print("    "+str(g))
        elif data_type == "PLAN":
            print("  PLAN: "+str(data))
        elif data_type == "ANOMALY":
            print("  ANOMALY: " + str(data))
        elif data_type == "ACTION":
            print("  ACTION: "+str(data))
        elif data_type == "REMOVED GOAL":
            print("  REMOVED GOAL: "+str(data))
        else:
            print("  UNKNOWN_DATA_TYPE '" + data_type + "' : "+str(data))
            
    def printtrace(self):
        for cycle in self.trace.keys():
            for phase in self.trace[cycle].keys():
                print("---------------------------------------------\n[cycle "+str(cycle)+"][phase "+str(phase)+"]\n---------------------------------------------\n\n")
                for datum in self.trace[cycle][phase]:
                    # datum[0] is type, datum[1] is actual data
                    self.print_data(datum[0],datum[1])
                    print("\n")

#    def gen_trace_graph(self):
        


