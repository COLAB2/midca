from MIDCA import base

class MRSimpleMonitor(base.BaseModule):

    def run(self, cycle, verbose = 2):
        self.mem.set(self.mem.TRACE_SEGMENT, self.get_last_phase())

    def get_last_phase(self):
        """ Return a small part of the trace """
        return self.mem.trace.get_data(self.mem.trace.get_current_cycle(), self.mem.trace.get_current_phase())
