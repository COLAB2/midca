from midca import ZmqInterface
import time
# incoming message handlers
class WorldHandler(ZmqInterface.IncomingMsgHandler):

    '''
    class that receives messages from the simulator about the states of the world
    '''

    def __init__(self, ip_address, connection, connection_type, midcaObject, memKey = None):
        super(WorldHandler, self).__init__(ip_address, connection, connection_type,
        midcaObject)
        if memKey:
            self.memKey = memKey
        else:
            self.memKey = self.mem.ZMQ_OBJS_DETECTED

    def run(self):
        while (True):
            msg = self.recieve_messages()
            if msg:
                self.mem.add(self.memKey, msg)
            time.sleep(0.1)

class RequestHandler(ZmqInterface.IncomingMsgHandler):

    '''
    class that receives messages from the simulator about the states of the world
    '''

    def __init__(self, ip_address, connection, connection_type, midcaObject, memKey = None):
        super(RequestHandler, self).__init__(ip_address, connection, connection_type,
        midcaObject)
        if memKey:
            self.memKey = memKey
        else:
            self.memKey = self.mem.ZMQ_REQS_DETECTED

    def run(self):
        while (True):
            msg = self.recieve_messages()
            if msg:
                self.mem.add(self.memKey, msg)
            time.sleep(0.1)

# outgoing message handlers
class SimulationHandler(ZmqInterface.OutgoingMsgHandler):

    '''
    class that receives messages from the simulator about the states of the world
    '''

    def __init__(self, ip_address, connection, connection_type, midcaObject, memKey = None):
        super(SimulationHandler, self).__init__(ip_address, connection, connection_type,
        midcaObject)
        if memKey:
            self.memKey = memKey
        else:
            self.memKey = self.mem.ZMQ_OUTGOING_MSGS

    def run(self):
        while (True):
            outRequests = self.mem.get_and_clear(self.memKey)
            if outRequests:
                for message in outRequests:
                    self.send_messages(message)

class OutReqHandler(ZmqInterface.OutgoingMsgHandler):

    '''
    class that receives messages from the simulator about the states of the world
    '''

    def __init__(self, ip_address, connection, connection_type, midcaObject, memKey = None):
        super(OutReqHandler, self).__init__(ip_address, connection, connection_type,
        midcaObject)
        if memKey:
            self.memKey = memKey
        else:
            self.memKey = self.mem.ZMQ_OUTGOING_REQS

    def run(self):
        while (True):
            outRequests = self.mem.get_and_clear(self.memKey)
            if outRequests:
                for message in outRequests:
                    self.send_messages(message)
