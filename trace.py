from __future__ import print_function
from MIDCA.mem import Memory
from MIDCA.modules import planning2
import copy
import shlex, subprocess

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


    def data_str(self, data_type, data):
        result_str = ""
        if data_type == "WORLD":
            result_str = "  WORLD: "+str(data)
        elif data_type == "PREV WORLD":
            result_str = "  PREV WORLD: "+str(data)
        elif data_type == "CURR WORLD":
            result_str = "  CURR WORLD: "+str(data)
        elif data_type == "GOALS":
            if data is None:
                result_str = "  GOALS: []"
            else:
                result_str = "  GOALS: "
                for g in data:
                    result_str += "    "+str(g)
        elif data_type == "PLAN":
            result_str = "  PLAN: "+str(data)
        elif data_type == "ANOMALY":
            result_str = "  ANOMALY: " + str(data)
        elif data_type == "ACTION":
            result_str = "  ACTION: "+str(data)
        elif data_type == "REMOVED GOAL":
            result_str = "  REMOVED GOAL: "+str(data)
        else:
            result_str = "  UNKNOWN_DATA_TYPE '" + data_type + "' : "+str(data)

        return result_str
            
    def printtrace(self):
        for cycle in self.trace.keys():
            for phase in self.trace[cycle].keys():
                print("---------------------------------------------\n[cycle "+str(cycle)+"][phase "+str(phase)+"]\n---------------------------------------------\n\n")
                for datum in self.trace[cycle][phase]:
                    # datum[0] is type, datum[1] is actual data
                    print(self.data_str(datum[0],datum[1]))
                    print("\n")

    def writeToPDF(self, pdf_filename="trace.pdf"):
        """Requires the 'dot' command be installed on the current system. To
        install on unix simply type 'sudo apt-get install
        graphviz'
        
        The filename must end in .pdf . A temporary .dot
        file will be made and then removed to create the
        pdf file. The path for the pdf file will be the
        same for the .dot file.
        
        Since this function traverses the graph, it is
        important that there is not a cycle. If there is,
        then one of the relationships in the cycle may not
        described in the graph
        
        Note that this could create a potential security
        vulnerability if the filename of the pdf passed in
        is prepended with malicious code.
        
        To-do list:
        - put everything in a directory
        
        """
        
        assert(pdf_filename.endswith(".pdf"))
        
        # get the filename for dot by removing '.pdf'
        dotfilename = copy.deepcopy(pdf_filename[0:-4]) + ".dot"
        dotfilestr = "digraph\n{\n"
        graphstr = "" # for making dependency graph
        prev_node_id = "" # for making dependency graph
        for cycle in self.trace.keys():
            for phase in self.trace[cycle].keys():
                curr_node_id = " C" + str(cycle) + "P"+ str(phase)
                
                # generate the string for this node
                for datum in self.trace[cycle][phase]:
                    # datum[0] is type, datum[1] is actual data
                    curr_node_label = curr_node_id +"\n"+str(self.data_str(datum[0],datum[1]))
                    dotfilestr += curr_node_id +" [label=\""+curr_node_label+" \"]\n"                    
                # generate string for dependency graph
                if prev_node_id is "":
                    # this is first iteration, no dependency added
                    prev_node_id = curr_node_id
                else:
                    graphstr += prev_node_id + " -> " + curr_node_id + " \n"
                    prev_node_id = curr_node_id

        dotfilestr += "\n"

        dotfilestr += graphstr # add dependency graph
                
        dotfilestr += "\n}\n"
        f = open(dotfilename, 'w')
        f. write(dotfilestr)
        f.close()
        print("Wrote dot file to " + dotfilename)
        genPDFCommand = "dot -Tpdf "+ dotfilename + " -o " + pdf_filename
        print("genPDFCommand = " + genPDFCommand)
        dot_output = subprocess.check_output(shlex.split(genPDFCommand))
        print("dot_output = " + str(dot_output))
        #subprocess.call(shlex.split("rm "+dotfilename))
        print("Drawing of current trace written to " + pdf_filename)
    

#    def gen_trace_graph(self):
        


