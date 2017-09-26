from midca import base

class MRSimpleMonitor(base.BaseModule):

    

    def run(self, cycle, verbose = 2):
        print("*********************IN META MONITOR*******************")
        self.verbose = verbose
        self.mem.set(self.mem.TRACE_SEGMENT, [self.mem.trace.get_current_phase(),self.get_last_phase()])
        
        if self.verbose >= 1: print("    Retrieved last segment of trace") #+str(self.mem.get(self.mem.TRACE_SEGMENT)))
        if self.verbose >= 1: print "  "+str(self.mem.get(self.mem.TRACE_SEGMENT))

    def get_last_phase(self):
        """ Return a small part of the trace """
        return self.mem.trace.get_data(self.mem.trace.get_current_cycle(), self.mem.trace.get_current_phase())
