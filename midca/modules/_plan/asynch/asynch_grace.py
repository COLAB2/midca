from midca import rosrun, plans
from midca import midcatime
import traceback
import math
import copy
import numpy as np
import time


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
verbose = 1

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
            #actions.append(SurveyCellErgodic(mem, midcaAction))
            actions.append(SurveyCell(mem, midcaAction))

        elif midcaAction.op == "ascend":
            actions.append(GraceAscend(mem, midcaAction))

        elif midcaAction.op == "descend":
            actions.append(GraceDescend(mem, midcaAction))


        elif midcaAction.op == "glideback":
            actions.append(GraceGlide(mem, midcaAction))

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
            import sys
            sys.exit()

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
        self.interface = self.mem.get(self.mem.INTERFACE)
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
        tagworld = self.interface.TagWorld()
        time = tagworld.simtime()
        tagworld = self.interface.TagWorld()
        tagworld.endSim()
        #time = tagworld.simtime()
        print ("Experiment Completed : Hotspot found at time : " + str(time))
        raise SystemExit
        pass

    def check_confirmation(self):# read a file output by program chechinkg for surface and return true or false
        if self.skip:
            self.skip = False
            return False


        return True

class GraceAscend(AsynchAction):
    '''
    Action that communicates it's depth to fumin
    '''

    def __init__(self, mem, midcaAction):
        self.mem = mem
        self.interface = self.mem.get(self.mem.INTERFACE)
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
        tagworld = self.interface.TagWorld()
        tagworld.UpdateSurfstatus("ascend")
        time.sleep(0.2)
        pass

    def check_confirmation(self):# read a file output by program chechinkg for surface and return true or false
        if self.skip:
            self.skip = False
            return False
        return True


class GraceDescend(AsynchAction):
    '''
    Action that communicates it's depth to fumin
    '''

    def __init__(self, mem, midcaAction):
        self.mem = mem
        self.interface = self.mem.get(self.mem.INTERFACE)
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
        time.sleep(0.2)
        tagworld = self.interface.TagWorld()
        tagworld.UpdateSurfstatus("descend")
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
        self.action = midcaAction
        self.mem = mem
        self.interface = self.mem.get(self.mem.INTERFACE)
        self.time = None
        self.complete = False
        self.skip = True
        self.skip = 5
        self.skiponce = 1
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

        tagworld = self.interface.TagWorld()
        tagworld.move_cell([0,0], go_to_location)
    """
    def check_confirmation(self):
        world = self.mem.get(self.mem.STATES)[-1]
        #argnames = [str(arg) for arg in self.action.args]
        #atoms = world.get_atoms(filters=["certainblocRadius", "grace", argnames[1]])
        #if not atoms:
        #    self.status = FAILED
        #    return False
        argnames = [str(arg) for arg in self.action.args]
        go_to_location = self.parse_tile(argnames[2])
        tagworld = self.interface.TagWorld()
        current_position = tagworld.get_cell()
        if current_position:
            current_position = current_position.split(",")
            current_position = [int(position) for position in current_position]
            if current_position == go_to_location:
                return True
            #else:
            #   self.implement_action()

        return False
    """
    def check_confirmation(self):
        # skip 5 cells
        if self.skip:
            self.skip -= 1
            return False

        #world = self.mem.get(self.mem.STATES)[-1]
        #argnames = [str(arg) for arg in self.action.args]
        #atoms = world.get_atoms(filters=["certainblocRadius", "grace", argnames[2]])
        #if not atoms:
        #    self.status = FAILED
        #    return False

        tagworld = self.interface.TagWorld()
        confirmation = tagworld.searchComplete()
        if confirmation == str(True) and self.skiponce:
            #time.sleep(7)
            self.skiponce += -1
            return False
        if confirmation == str(True):
            #self.mem.set(self.mem.GETDATA, True)
            #time.sleep(1)
            return True
        return False


class SurveyCell(AsynchAction):
    '''
    Action to make grace reach surface
    '''

    def __init__(self, mem, midcaAction):
        self.action = midcaAction
        self.mem = mem
        self.interface = self.mem.get(self.mem.INTERFACE)
        self.time = None
        self.complete = False
        self.skip = 5
        self.skiponce = 1
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
        tagworld = self.interface.TagWorld()
        tagworld.search(go_to_location)

    def check_confirmation(self):
        # skip 5 cells
        if self.skip:
            self.skip -= 1
            return False

        #world = self.mem.get(self.mem.STATES)[-1]
        #argnames = [str(arg) for arg in self.action.args]
        #atoms = world.get_atoms(filters=["certainblocRadius", "grace", argnames[2]])
        #if not atoms:
        #    self.status = FAILED
        #    return False

        tagworld = self.interface.TagWorld()
        confirmation = tagworld.searchComplete()
        if confirmation == str(True) and self.skiponce:
            #time.sleep(7)
            self.skiponce += -1
            return False
        if confirmation == str(True):
            self.mem.set(self.mem.GETDATA, True)
            #time.sleep(1)
            return True
        return False

class SurveyCellErgodic(AsynchAction):
    '''
    Action to make grace reach surface
    '''

    def __init__(self, mem, midcaAction):
        self.action = midcaAction
        self.mem = mem
        self.interface = self.mem.get(self.mem.INTERFACE)
        self.time = None
        self.complete = False
        self.skip = 5
        self.skiponce = 1
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
        tagworld = self.interface.TagWorld()
        tagworld.searchErgodic(go_to_location)

    def check_confirmation(self):
        # skip 5 cells
        if self.skip:
            self.skip -= 1
            return False

        #world = self.mem.get(self.mem.STATES)[-1]
        #argnames = [str(arg) for arg in self.action.args]
        #atoms = world.get_atoms(filters=["certainblocRadius", "grace", argnames[2]])
        #if not atoms:
        #    self.status = FAILED
        #    return False

        tagworld = self.interface.TagWorld()
        confirmation = tagworld.searchErgodicComplete()
        if confirmation == str(True) and self.skiponce:
            #time.sleep(7)
            self.skiponce += -1
            return False

        if confirmation == str(True):
            self.mem.set(self.mem.GETDATA, True)
            #time.sleep(1)
            return True
        return False


class GraceGlide(AsynchAction):
    '''
    Action that communicates it's depth to fumin
    '''

    def __init__(self, mem, midcaAction):
        self.mem = mem
        self.interface = self.mem.get(self.mem.INTERFACE)
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
        tagworld = self.interface.TagWorld()
        tagworld.remove_remora()
        #time.sleep(0.2)
        pass

    def check_confirmation(self):
        if self.skip:
            self.skip = False
            return False

        tagworld = self.interface.TagWorld()
        status = tagworld.remove_remora_status()
        if status == "True":
            return True

        #self.implement_action()

        return False
