class MRSimpleMonitor:

    def __init__(self, verbose = 0):
        # TODO get trace from mem
        #self.trace = trace
        self.verbose = verbose

    def get_last_phase(self):
        """ Return a small part of the trace """
        return self.trace.get_data(self.trace.get_current_cycle(), self.trace.get_current_phase())
