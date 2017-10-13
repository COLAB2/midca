from __future__ import print_function
import copy
import shlex, subprocess # used for generating a pdf of the trace
from collections import OrderedDict

"""
How-To:
1. In each module, before storing data into the trace call add_module() then
2. for each piece of data you want to add into the trace, call add_data()
"""

MAX_TRACE_SIZE = 100

class CogTrace:
    # trace[<cycle>][<module-id>] returns a list of what happened in
    # that module in that cycle

    def __init__(self):
        self.trace = {}
        self.cycle = -1 # current cycle
        self.module = "" # current module
        self.all_modules = OrderedDict() # alternative structure of the same data as self.trace

    def add_module(self, cycle, module):
        """args:
           cycle := integer representing a cycle
           module := string representing the name of a module
        """
        if self.all_modules and len(self.all_modules) > MAX_TRACE_SIZE:
            i = MAX_TRACE_SIZE - 50
            for j in range(i):
                self.all_modules.popitem(last=False)
        
        trace_cycle_keys = self.trace.keys()
        if len(trace_cycle_keys) > MAX_TRACE_SIZE:
            min_key = min(trace_cycle_keys)
            del self.trace[min_key]
             
        if cycle in self.trace.keys():
            if len(self.trace[cycle]) > 0 and module in self.trace[cycle].keys():
                #print("Changing data for module "+str(module)+" in cycle "+str(cycle)+ " to " + str(data))
                self.trace[cycle][module] = []
            else:
                #print("Fresh insert of data for module "+str(module)+" in cycle "+str(cycle)+ " to " + str(data))
                self.trace[cycle][module] = []
        else:
            self.trace[cycle] = OrderedDict()
            #print("Fresh insert of data for module "+str(module)+" in cycle "+str(cycle)+ " to " + str(data))
            self.trace[cycle][module] = []

        self.cycle = cycle
        self.module = module
        self.all_modules[(cycle,module)] = []
        

    def add_data(self, data_type, data):
        """
        data_type is one of "WORLD, GOALS, etc"
        """
        
        if self.cycle != -1 and self.module != "":
            self.trace[self.cycle][self.module].append([data_type,data])
            self.all_modules[(self.cycle,self.module)].append([data_type,data])
        
    def get_data(self, cycle, phase):
        if cycle < 0:
            return [] # if not initialized, no data to return
        else:
            # check module
            if phase not in self.trace[cycle]:
                return []

            return self.trace[cycle][phase]

    def get_current_phase_data(self):
        '''
        Returns the data of the most recent phase stored in the trace.
        Equivalent to calling: get_data(get_current_cycle(), get_current_phase())
        '''
        return self.get_data(self.get_current_cycle(), self.get_current_phase())

    def get_current_cycle(self):
        return self.cycle

    def get_current_phase(self):
        return self.module

    def get_n_prev_phase(self,n=0):
        '''
        returns the nth previous phase, if n is 1, get the second to last phase executed, etc
        '''
        try:
            if len(self.all_modules) > n:
                return self.all_modules.items()[-(n+1)]
        
        except :
            print("problem in get_n_prev_phase")

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
            result_str = "  UNKNOWN_DATA_TYPE '" + str(data_type) + "' : "+str(data)

        return result_str

    def printtrace(self):
        for cycle in self.trace.keys():
            for phase in self.trace[cycle].keys():
                print("---------------------------------------------\n[cycle "+str(cycle)+"][module "+str(phase)+"]\n---------------------------------------------\n\n")
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
                curr_node_id = " C_" + str(cycle) + "_P_"+ str(phase)

                # generate the string for this node
                curr_node_label = curr_node_id +"\n"
                for datum in self.trace[cycle][phase]:
                    #print("[writetopdf] self.trace[cycle][phase] is "+str(self.trace[cycle][phase]))
                    # datum[0] is type, datum[1] is actual data
                    curr_node_label += str(self.data_str(datum[0],datum[1]))+"\n"
                    dotfilestr += curr_node_id +" [shape=rect label=\""+curr_node_label+" \"]\n"
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


