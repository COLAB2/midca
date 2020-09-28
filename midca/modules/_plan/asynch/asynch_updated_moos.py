from midca import rosrun, plans
from midca import midcatime
import traceback
import math
import copy
import numpy as np
import zmq, time
import numpy

try:
    # moos specific import to perform apprehend and report actions
    from midca.domains.moos_domain import moosworld
except:
    print ("Moos specific imports failed")


END_COLOR_CODE = '\033[0m'
NOT_STARTED = 0
NS_COLOR_CODE = END_COLOR_CODE
IN_PROGRESS = 1
IP_COLOR_CODE = '\033[92m'
COMPLETE = 2
C_COLOR_CODE = '\033[94m'
FAILED = 3
F_COLOR_CODE = '\033[91m'

#set this to change output for all asynch actions.
verbose = 2


def get_asynch_action(midcaAction):
    raise ArgumentException("midca action " + str(midcaAction)  + " does not translate to a \
    valid asynchronous action.")

def asynch_plan(mem, midcaPlan):
    '''
    returns an asynchronous plan that corresponds to the given MIDCA plan.
    '''
    actions = []
    goals = midcaPlan.goals
    for midcaAction in midcaPlan.actions:
        if midcaAction.op == "ignore":
            actions.append(IgnoreMine(mem, midcaAction))

        elif midcaAction.op == "avoid":
            actions.append(AvoidMine(mem, midcaAction))

        elif midcaAction.op == "remove":
            actions.append(RemoveMine(mem, midcaAction))

        elif midcaAction.op == "transit":
            actions.append(transit(mem, midcaAction))

        elif midcaAction.op == "do-clear":
            actions.append(doclear(mem, midcaAction))

        else:
            if verbose >= 1:
                print "MIDCA action", midcaAction, "does not correspond to an asynch",
                "action. MIDCA will skip this action"

    return AsynchPlan(actions, goals)

class AsynchPlan(plans.Plan):

    '''
    subclass of MIDCA Plan class that uses asynchronous actions.
    '''

    def finished(self):
        '''
        overrides plan.finished(). Declares a plan complete if all its actions report
        complete or failed.
        '''
        for action in self.actions:
            if action.status != COMPLETE and action.status != FAILED:
                return False
        return True

    @property
    def status(self):
        '''
        property that returns the plan's status. This can be NOT_STARTED, IN_PROGRESS,
        FAILED, or COMPLETE. If any action fails, the plan is considered to have failed.
        The plan is complete when all actions are complete.
        '''
        status = COMPLETE
        for action in self.actions:
            if action.status == FAILED:
                return FAILED
            elif action.status == NOT_STARTED and status == COMPLETE:
                status = NOT_STARTED
            elif action.status == IN_PROGRESS:
                status = IN_PROGRESS
        return status

    def __str__(self):
        s = ""
        for action in self.actions:
            if action.status == NOT_STARTED:
                s += NS_COLOR_CODE
            elif action.status == IN_PROGRESS:
                s += IP_COLOR_CODE
            elif action.status == FAILED:
                s += F_COLOR_CODE
            elif action.status == COMPLETE:
                s += C_COLOR_CODE

            s += str(action) + " "
        return s[:-1] + END_COLOR_CODE

class AsynchAction:

    nextID = 0

    def __init__(self, mem, midcaAction, executeFunc, isComplete, blocks):
        self.status = NOT_STARTED
        self.mem = mem
        self.midcaAction = midcaAction
        self.executeFunc = executeFunc
        self.isComplete = isComplete
        self.blocks = blocks
        self.startTime = None
        self.id = AsynchAction.nextID
        AsynchAction.nextID += 1

    def execute(self):
        if not self.startTime:
            self.startTime = midcatime.now()
        self.status = IN_PROGRESS
        if not self.executeFunc:
            return
        try:
            self.executeFunc(self.mem, self.midcaAction, self.status)
        except:
            if verbose >= 2:
                print "Error executing action", self, ":\n", traceback.format_exc(),
                "\n\nAction assumed to be failed"
            self.status = FAILED

    def check_complete(self):
        if not self.startTime:
            self.startTime = midcatime.now()
        try:
            complete = self.isComplete(self.mem, self.midcaAction, self.status)
            if verbose >= 2 and not complete:
                print "Action", self, "not complete."
            if verbose >= 1 and complete:
                print "Action", self, "complete."
            if complete:
                self.status = COMPLETE
            return complete
        except:
            if verbose >= 1:
                print "Error checking completion status for action", self, " - Assuming \
                 failure:\n", traceback.format_exc()
            self.status = FAILED

    def ros_msg(self, topic, d):
        '''
        arg d should be a dictionary that contains the key/value pairs to be sent.
        '''
        sent = rosrun.send_msg(topic, rosrun.dict_as_msg)
        if not sent:
            if verbose >= 1:
                print "Unable to send msg; ", d, "on topic", topic, " Action", self,
                "assumed failed."
            self.status = FAILED

    def __str__(self):
        return str(self.midcaAction)

class IgnoreMine(AsynchAction):

    '''
    Grace action that senses depth
    '''

    def __init__(self, mem, midcaAction):
        # zeromq connection
        context = zmq.Context()
        self.publisher = context.socket(zmq.PUB)
        self.publisher.connect("tcp://127.0.0.1:5999")

        # initialize memory
        self.mem = mem
        self.midcaAction = midcaAction
        self.skip = True
        self.complete = False
        executeAction = lambda mem, midcaAction, status: self.implement_action()
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation()
        AsynchAction.__init__(self, mem, midcaAction, executeAction,
        completionCheck, True)

    def implement_action(self):
        time.sleep(0.25)
        for i in range(2):
            self.publisher.send_multipart([b"Vehicle", b"speed =0.0"])

    def check_confirmation(self):
        if self.skip:
            self.skip = False
            return False
        time.sleep(0.25)
        return True

class AvoidMine(AsynchAction):

    '''
    Grace action that senses depth
    '''

    def __init__(self, mem, midcaAction):
        # zeromq connection
        context = zmq.Context()
        self.publisher = context.socket(zmq.PUB)
        self.publisher.connect("tcp://127.0.0.1:5999")

        # variable to skip 1 cycle to execute this action
        self.skip = True

        # initialize memory
        self.mem = mem
        self.midcaAction = midcaAction
        self.complete = False
        executeAction = lambda mem, midcaAction, status: self.implement_action(midcaAction)
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation()
        AsynchAction.__init__(self, mem, midcaAction, executeAction,
        completionCheck, True)

    def implement_action(self, action):
        time.sleep(0.1)
        self.publisher.send_multipart([b"Vehicle", b"speed =0.0"])
        time.sleep(0.3)
        x = self.mem.get(self.mem.AGENT_LOCATION)[0]
        y = str(self.mem.get(self.mem.AGENT_LOCATION)[1] - 3)
        x = str(x - 20)
        message = [b"Vehicle", b"points = pts={17,-15:5,-33} # speed= 0.3"]
        self.publisher.send_multipart(message)

    def check_confirmation(self):
        speed = self.mem.get(self.mem.REMUS_SPEED)
        if self.mem.get(self.mem.AGENT_LOCATION)[3].lower() == "clear" and speed >0.1:
            self.mem.set(self.mem.PAUSE, True)
            self.skip = False

        if self.skip:
            return False

        if (self.mem.get(self.mem.AGENT_LOCATION)[3]).lower() == "nothingtodo":
            return True

class RemoveMine(AsynchAction):

    '''
    Grace action that senses depth
    '''

    def __init__(self, mem, midcaAction):
        # zeromq connection
        context = zmq.Context()
        self.publisher = context.socket(zmq.PUB)
        self.publisher.connect("tcp://127.0.0.1:5999")

        # variable to skip 1 cycle to execute this action
        self.skip = True

        # initialize memory
        self.mem = mem
        self.midcaAction = midcaAction
        self.complete = False
        executeAction = lambda mem, midcaAction, status: self.implement_action(midcaAction)
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation()
        AsynchAction.__init__(self, mem, midcaAction, executeAction,
        completionCheck, True)

    def implement_action(self, action):
        time.sleep(0.3)
        label= int(action.args[0].replace("mine",""))
        self.publisher.send_multipart([b"RemoveHazard", str(label)])
        for i in range(2):
            self.publisher.send_multipart([b"Vehicle", b"speed =0.0"])
            time.sleep(0.1)
        # assign scores
        if (action.args[1] == "ga1" or action.args[1] == "ga2"):
            score = self.mem.get(self.mem.MOOS_SCORE) + 1
            self.mem.set(self.mem.MOOS_SCORE, score)

        elif (action.args[1] == "qroute"):
            score = self.mem.get(self.mem.MOOS_SCORE) + 1
            self.mem.set(self.mem.MOOS_SCORE, score)
        else:
            pass

    def check_confirmation(self):
        if self.skip:
            self.skip = False
            return False
        time.sleep(0.3)
        return True

class transit(AsynchAction):

    '''
    Grace action that senses depth
    '''

    def __init__(self, mem, midcaAction):
        # zeromq connection
        context = zmq.Context()
        self.publisher = context.socket(zmq.PUB)
        self.publisher.connect("tcp://127.0.0.1:5999")
        self.skip = 2
        # initialize memory
        self.mem = mem
        self.midcaAction = midcaAction
        self.complete = False
        executeAction = lambda mem, midcaAction, status: self.implement_action(midcaAction)
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation(midcaAction)
        AsynchAction.__init__(self, mem, midcaAction, executeAction,
        completionCheck, True)

    def implement_action(self, action):

        time.sleep(0.3)

        argnames = [str(arg) for arg in action.args]

        if ("transit-area-1" in argnames[2:]):
            message = [b"Vehicle", b"point = 11,-3 # speed= 0.5"]
            self.publisher.send_multipart(message)

        elif ("ga1" in argnames[2:]):
            message = [b"Vehicle", b"point = 1,-60 # speed= 0.5"]
            self.publisher.send_multipart(message)

        elif ("ga2" in argnames[2:]):
            message = [b"Vehicle", b"point = 131,-60 # speed= 0.5"]
            self.publisher.send_multipart(message)

        elif ("qroute-transit-area-1" in argnames[2:]):
            message = [b"Vehicle", b"point = 77,-63 # speed= 0.5"]
            for i in range(2):
                self.publisher.send_multipart(message)

        elif ("home1" in argnames):
            message = [b"Vehicle", b"point = 165,-6 # speed= 0.5"]
            for i in range(2):
                    self.publisher.send_multipart(message)

        elif ("transit-area-2" in argnames):
            message = [b"Vehicle", b"point = 133,-30 # speed= 0.5"]
            for i in range(2):
                self.publisher.send_multipart(message)


    def check_confirmation(self, action):
        speed = self.mem.get(self.mem.REMUS_SPEED)

        # skip 2 cycles
        if self.skip >= 0:
            self.skip -= 1
            time.sleep(0.1)
            return False

        if (self.mem.get(self.mem.AGENT_LOCATION)[3]).lower() == "nothingtodo":
            return True

        if self.mem.get(self.mem.AGENT_LOCATION)[3].lower() == "clear" and speed == 0:
                self.publisher.send_multipart([b"Vehicle", b"speed = 0.5"])
                self.skip = 2

        return False

class doclear(AsynchAction):

    '''
    Grace action that senses depth
    '''

    def __init__(self, mem, midcaAction):
        # zeromq connection
        context = zmq.Context()
        self.publisher = context.socket(zmq.PUB)
        self.publisher.connect("tcp://127.0.0.1:5999")

        # initialize memory
        self.mem = mem
        self.midcaAction = midcaAction
        self.complete = False
        self.skip = 2
        executeAction = lambda mem, midcaAction, status: self.implement_action(midcaAction)
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation(midcaAction)
        AsynchAction.__init__(self, mem, midcaAction, executeAction,
        completionCheck, True)

    def implement_action(self, action):

        time.sleep(0.25)

        argnames = [str(arg) for arg in action.args]

        if ("ga1" in argnames):
            message = [b"Vehicle",b" points=format=lawnmower,label=dedley_survey, x=20, y=-80, width=30, height = 30,lane_width=10, rows=north-south,degs=0 # speed =0.4"]
            print ("Start Surveying ....")
            for i in range(2):
                self.publisher.send_multipart(message)

        if ("ga2" in argnames):
            message = [b"Vehicle",b" points=format=lawnmower,label=dedley_survey, x=150, y=-80, width=30, height = 30,lane_width=10, rows=north-south,degs=0 # speed =0.4"]
            for i in range(2):
                self.publisher.send_multipart(message)

    def check_confirmation(self, action):
        speed = self.mem.get(self.mem.REMUS_SPEED)

        # skip 2 cycles
        if self.skip >=0:
            self.skip -= 1
            time.sleep(0.1)
            return False

        if (self.mem.get(self.mem.AGENT_LOCATION)[3]).lower() == "nothingtodo":
            return True

        if (self.mem.get(self.mem.AGENT_LOCATION)[3]).lower() == "clear" and speed == 0:
                self.publisher.send_multipart([b"Vehicle", b"speed = 0.4"])
                self.skip = 2


        return False


