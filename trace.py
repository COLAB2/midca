from __future__ import print_function


class CogTrace:

    # trace[<cycle>][<module-id>] returns a list of what happened in
    # that module in that cycle
    trace = {}

    
    def addphase(self, cycle, phase, data):
    """args: 
       cycle := integer representing a cycle
       phase := string representing the name of a phase """
    
    if self.trace[cycle]:
        if self.trace[cycle][phase]:
            print "Changing data for phase "+str(phase)+" in cycle "+str(cycle)+ " to " + str(data)
            self.trace[cycle][phase] = data
        else:
            print "Fresh insert of data for phase "+str(phase)+" in cycle "+str(cycle)+ " to " + str(data)
            self.trace[cycle][phase] = data            
    else:
        self.trace[cycle] = {}
            print "Fresh insert of data for phase "+str(phase)+" in cycle "+str(cycle)+ " to " + str(data)
            self.trace[cycle][phase] = data                    

    def printtrace(self):
        for cycle in self.trace.keys():
            for phase in self.trace[cycle].keys():
                print "[cycle "+str(cycle)+"][phase "+str(phase)+"] "+str(data)
