from midca import rosrun, plans
from midca import midcatime
import traceback
import math
import copy
import numpy as np
import zmq, time


try:
    from geometry_msgs.msg import PointStamped
    from geometry_msgs.msg import Point
    from MIDCA.examples import ObjectDetector
    from std_msgs.msg import String
    from scipy.spatial import distance
except:
    pass  # if ROS is not installed, an error message will already have been generated.

END_COLOR_CODE = '\033[0m'
NOT_STARTED = 0
NS_COLOR_CODE = END_COLOR_CODE
IN_PROGRESS = 1
IP_COLOR_CODE = '\033[92m'
COMPLETE = 2
C_COLOR_CODE = '\033[94m'
FAILED = 3
F_COLOR_CODE = '\033[91m'

FEEDBACK_KEY = "code"
CMD_ID_KEY = "cmd_id"
POINT_TOPIC = "point_cmd"
LOC_TOPIC = "loc_cmd"
GRAB_TOPIC = "grabbing_cmd"
RAISE_TOPIC = "raise_cmd"
RELEASE_TOPIC = "release_cmd"
# set this to change output for all asynch actions.
verbose = 2

MAX_SIGHTING_LAG = 3.0
MAX_SIGHTING_WAIT = 5.0


def get_asynch_action(midcaAction):
    raise ArgumentException("midca action " + str(midcaAction) + " does not translate to a \
    valid asynchronous action.")


def asynch_plan(mem, midcaPlan):
    '''
    returns an asynchronous plan that corresponds to the given MIDCA plan.
    '''
    actions = []
    goals = midcaPlan.goals
    for midcaAction in midcaPlan.actions:
        if midcaAction.op == "communicate":
            actions.append(GraceCommunicate(mem, midcaAction))

        elif midcaAction.op == "movenorth" or \
             midcaAction.op == "moveeast" or \
             midcaAction.op == "movesouth" or \
             midcaAction.op == "movewest":
            actions.append(MoveToCell(mem, midcaAction))

        elif midcaAction.op == "collectdata":
            actions.append(SurveyCell(mem, midcaAction))

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
        if not self.check_complete:
            return
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

class GraceCommunicate(AsynchAction):
    '''
    Action that communicates it's depth to fumin
    '''

    def __init__(self, mem, midcaAction):
        self.mem = mem
        self.time = None
        self.skip = True
        self.depth = None
        self.action = midcaAction
        self.complete = False
        executeAction = lambda mem, midcaAction, status: self.implement_action()
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation()
        AsynchAction.__init__(self, mem, midcaAction, executeAction,
                              completionCheck, True)

    def implement_action(self):
       pass

    def check_confirmation(self):# read a file output by program chechinkg for surface and return true or false
        if self.skip:
            self.skip = False
            return False
        return True


class MoveToCell(AsynchAction):
    '''
    Action to make grace reach surface
    '''

    def __init__(self, mem, midcaAction):
        context = zmq.Context()
        self.publisher = context.socket(zmq.PUB)
        self.publisher.connect("tcp://127.0.0.1:9560")

        self.start_pos = [-100, 36]
        self.distance =80
        self.action = midcaAction
        self.mem = mem
        self.time = None
        self.complete = False
        self.skip = True
        executeAction = lambda mem, midcaAction, status: self.implement_action()
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation()
        AsynchAction.__init__(self, mem, midcaAction, executeAction,
                              completionCheck, True)

    def parse_tile(self, input):
        output = []
        y_index = input.index('y')
        output = [int(input[2:y_index]) , int(input[y_index+1:])]
        return output

    def implement_action(self):
        self.time = midcatime.now()
        argnames = [str(arg) for arg in self.action.args]
        # which is the to go location for grace moveeast(grace, Tx3y3, Tx4y4)
        go_to_location = self.parse_tile(argnames[2])
        x = self.start_pos[0] + go_to_location[0] * self.distance
        y = self.start_pos[1] - go_to_location[1] * self.distance
        x = str(x)
        y = str(y)
        message = [b"M", b"point = "+x+","+y+" # speed= 0.5"]
        for i in range(2):
            time.sleep(0.1)
            self.publisher.send_multipart(message)

    def check_confirmation(self):
        if self.skip:
            self.skip = False
            time.sleep(2)
            return False
        if self.time:
            self.time = False
            return False
        if self.mem.get(self.mem.REMUS_SPEED) == 0.0:
            return True
        return False

class SurveyCell(AsynchAction):
    '''
    Action to make grace reach surface
    '''

    def __init__(self, mem, midcaAction):
        context = zmq.Context()
        self.publisher = context.socket(zmq.PUB)
        self.publisher.connect("tcp://127.0.0.1:9560")

        self.start_pos = [-100, 36]
        self.distance =80
        self.action = midcaAction
        self.mem = mem
        self.time = None
        self.complete = False
        self.skip = True
        executeAction = lambda mem, midcaAction, status: self.implement_action()
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation()
        AsynchAction.__init__(self, mem, midcaAction, executeAction,
                              completionCheck, True)

    def parse_tile(self, input):
        output = []
        y_index = input.index('y')
        output = [int(input[2:y_index]) , int(input[y_index+1:])]
        return output

    def implement_action(self):
        self.time = midcatime.now()
        argnames = [str(arg) for arg in self.action.args]
        # which is the to go location for grace collectdata(grace, sensordata, Tx4y4)
        go_to_location = self.parse_tile(argnames[2])
        x = self.start_pos[0] + go_to_location[0] * self.distance
        y = self.start_pos[1] - go_to_location[1] * self.distance
        x = str(x)
        y = str(y)
        message = [b"M",b"   points=format=lawnmower,label=dedley_survey, x="+x+", y="+y+", width=50, height = 50,lane_width=10, rows=north-south,degs=0  # speed =1"]
        for i in range(2):
            time.sleep(0.1)
            self.publisher.send_multipart(message)

    def check_confirmation(self):
        if self.skip:
            self.skip = False
            time.sleep(2)
            return False
        if self.time:
            self.time = False
            time.sleep(2)
            return False
        if self.mem.get(self.mem.REMUS_SPEED) == 0.0:
            return True
        return False
