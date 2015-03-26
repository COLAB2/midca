from __future__ import print_function


def singleton(self, instance={}):
    try:
        instance[self.__class__]
        instance[self.__class__] = self
    except KeyError:
        raise RuntimeError, "Instance already exists: %s" % self.__class__ 


class CogTrace:
    """ Singleton class so it can be used all over MIDCA """

    instance = None

    # trace[<cycle>][<module-id>] returns a list of what happened in
    # that module in that cycle
    trace = {}
    
    def __init__(self):
        singleton(self)

    def getInstance(self):
        if _CogTrace.instance == None:
            _CogTrace.instance = _CogTrace()
        return _CogTrace.instance
        
    def addphase(self, cycle, phase, data):
        """args: 
           cycle := integer representing a cycle
           phase := string representing the name of a phase """
    
        if self.trace[cycle]:
            if len(self.trace[cycle]) > 0 and self.trace[cycle][phase]:
                print("Changing data for phase "+str(phase)+" in cycle "+str(cycle)+ " to " + str(data))
                self.trace[cycle][phase] = data
            else:
                print("Fresh insert of data for phase "+str(phase)+" in cycle "+str(cycle)+ " to " + str(data))
                self.trace[cycle][phase] = data            
        else:
            self.trace[cycle] = {}
            print("Fresh insert of data for phase "+str(phase)+" in cycle "+str(cycle)+ " to " + str(data))
            self.trace[cycle][phase] = data                    

    def printtrace(self):
        for cycle in self.trace.keys():
            for phase in self.trace[cycle].keys():
                print("[cycle "+str(cycle)+"][phase "+str(phase)+"] "+str(data))
