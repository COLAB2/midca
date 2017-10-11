from MIDCA import rosrun, plans
from MIDCA import midcatime
import traceback
from MIDCA.examples.homography import * 
import math
import copy
import numpy as np


try:
    from geometry_msgs.msg import PointStamped
    from geometry_msgs.msg import Point
    from MIDCA.examples import ObjectDetector
    from std_msgs.msg import String
    from scipy.spatial import distance
except:
    pass #if ROS is not installed, an error message will already have been generated.


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
#set this to change output for all asynch actions.
verbose = 2

MAX_SIGHTING_LAG = 3.0
MAX_SIGHTING_WAIT = 5.0

def allowed_sighting_lag(objectID):
    '''
    returns how long ago an object can have been seen before MIDCA stops considering its
    location known
    '''
    return MAX_SIGHTING_LAG

def allowed_sighting_wait(objectID):
    '''
    returns the amount of midcatime MIDCA should wait to see an object before giving up.
    '''
    return MAX_SIGHTING_WAIT
    
'''
'''

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
        if midcaAction[0] == "block_until_seen":
            actions.append(AwaitCurrentLocation(mem, midcaAction, midcaAction[1], 
            allowed_sighting_lag(midcaAction[1]), allowed_sighting_wait(midcaAction[1])))
        elif midcaAction[0] == "point_to":
            cmdID = rosrun.next_id()
            actions.append(DoPoint(mem, midcaAction, midcaAction[1], 
            allowed_sighting_lag(midcaAction[1]), allowed_sighting_wait(midcaAction[1]),
            POINT_TOPIC, cmdID))
        
        elif midcaAction[0] == "reach_to_pickup":
            cmdID = rosrun.next_id()
            actions.append(DoReach(mem, midcaAction, midcaAction[1], 
            allowed_sighting_lag(midcaAction[1]), allowed_sighting_wait(midcaAction[1]),
            LOC_TOPIC, cmdID))    
        elif midcaAction[0] == "reach_to_unstack":
            cmdID = rosrun.next_id()
            actions.append(DoUnstack(mem, midcaAction, midcaAction[1], 
            allowed_sighting_lag(midcaAction[1]), allowed_sighting_wait(midcaAction[1]),
            LOC_TOPIC, cmdID))    
        elif midcaAction[0] == "grab":
            cmdID = rosrun.next_id()
            actions.append(DoGrab(mem, midcaAction, midcaAction[1], 
            allowed_sighting_lag(midcaAction[1]), allowed_sighting_wait(midcaAction[1]),
            GRAB_TOPIC, cmdID))
        elif midcaAction[0] == "release":
            cmdID = rosrun.next_id()
            actions.append(DoRelease(mem, midcaAction, midcaAction[1], 
            allowed_sighting_lag(midcaAction[1]), allowed_sighting_wait(midcaAction[1]),
            RELEASE_TOPIC, cmdID))    
        elif midcaAction[0] == "raising":
            cmdID = rosrun.next_id()
            actions.append(DoRaise(mem, midcaAction, midcaAction[1], 
            allowed_sighting_lag(midcaAction[1]), allowed_sighting_wait(midcaAction[1]),
            RAISE_TOPIC, cmdID))
        elif midcaAction[0] == "raising_arm":
            cmdID = rosrun.next_id()
            actions.append(DoRaise(mem, midcaAction, ' ', 
            allowed_sighting_lag(' '), allowed_sighting_wait(' '),
            RAISE_TOPIC, cmdID))
        elif midcaAction[0] == "putdown":
            cmdID = rosrun.next_id()
            actions.append(DoPut(mem, midcaAction, midcaAction[1], 
            allowed_sighting_lag(midcaAction[1]), allowed_sighting_wait(midcaAction[1]),
            LOC_TOPIC, cmdID))
        elif midcaAction[0] == "stack":
            cmdID = rosrun.next_id()
            actions.append(DoStack(mem, midcaAction, midcaAction[1], 
            allowed_sighting_lag(midcaAction[1]), allowed_sighting_wait(midcaAction[1]),
            RAISE_TOPIC, cmdID))    
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

def get_last_position(mem, objectOrID):
    state = mem.get(mem.STATE)
    positions = state.all_pos(objectOrID)
    if not positions:
        return None
    else:
        for state_pos in reversed(positions):
            if state_pos.position:
                return (state_pos.position)
    return None

def get_last_location(mem, objectOrID):
    world = mem.get(mem.STATE)
    sightings = world.all_sightings(objectOrID)
    if not sightings:
        return None
    else:
        for detectionEvent in reversed(sightings):
            if detectionEvent.loc:
                return (detectionEvent.loc, detectionEvent.time)
    return None

def check_2height_stack(mem,objectOrID):
    print("it is in check 3 height " )
    print(objectOrID)
    print("---------------------------------")
    print(get_last_position(mem,objectOrID))
    world = mem.get(mem.STATE)
    allobject = world.all_objects()
    for each in allobject:
        if(get_last_position(mem,objectOrID) == each):
            return each
    return False

def check_location_clear(mem,point,iterations):

    world = mem.get(mem.STATE)
    allobject = world.all_objects()
    x1 = point.x
    y1 = point.y
    z1 = point.z
    
    for each in allobject:
        lastLocReport = get_last_location(mem, each)

    for each in allobject:
        if (get_last_position(mem, each) == 'table'):
            lastLocReport = get_last_location(mem, each)
            x = lastLocReport[0].x 
            y = lastLocReport[0].y
	    z = lastLocReport[0].z
            d_point = (x , y,z)
            d1_point = (x1,y1,z1)
            total_distance = distance.euclidean(d_point,d1_point)
            if iterations == 1:
                if (total_distance < 0.10) :
                    point.x =  point.x + 0.076320692
                    point.y = point.y + 0.190633894
                    return point
            if iterations == 2:
                if (total_distance<0.10) :
                    point.x = point.x - 0.15271527
                    point.y = point.y - 0.01682536
                    return point
            
        
    return point




class AwaitCurrentLocation(AsynchAction):

    '''
    Action that blocks until there is a current (within last maxAllowedLag seconds)
    observation of the object's location.
    '''

    def __init__(self, mem, midcaAction, objectOrID, maxAllowedLag, maxDuration):
        self.objectOrID = objectOrID
        self.maxAllowedLag = maxAllowedLag
        self.maxDuration = maxDuration
        executeAction = None
        completionCheck = lambda mem, midcaAction, status: self.completion_check()
        AsynchAction.__init__(self, mem, midcaAction, executeAction, 
        completionCheck, True)
    
    def completion_check(self):
        t = midcatime.now()
        if t - self.startTime > self.maxDuration:
            if verbose >= 1:
                print "max midcatime exceeded for action:", self, "- changing status to failed." 
            self.status = FAILED
            return False
        lastLocReport = get_last_location(self.mem, self.objectOrID)
        if not lastLocReport:
            return False
        return t - lastLocReport[1] <= self.maxAllowedLag


class DoPoint(AsynchAction):
    
    '''
    Action that orders a point action. To 
    ensure success, an AwaitCurrentLocation action with <= the same maxAllowedLag and the 
    same target should be done immediately before this.
    '''
    
    def __init__(self, mem, midcaAction, objectOrID, maxAllowedLag, maxDuration, topic,
    msgID):
        self.objectOrID = objectOrID
        self.maxAllowedLag = maxAllowedLag
        self.maxDuration = maxDuration
        self.lastCheck = 0.0
        self.topic = topic
        self.complete = False
        self.msgID = msgID
        executeAction = lambda mem, midcaAction, status: self.send_point()
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation()
        AsynchAction.__init__(self, mem, midcaAction, executeAction, 
        completionCheck, True)
    
    def send_point(self):
        lastLocReport = get_last_location(self.mem, self.objectOrID)
        t = midcatime.now()
        if not lastLocReport:
            if verbose >= 1:
                print "No object location found, so action:", self, "will fail."
            self.status = FAILED
            return
        if t - lastLocReport[1] > self.maxAllowedLag:
            if verbose >= 1:
                print "Last object location report is too old -", 
                str((t - lastLocReport[1]).total_seconds()), "seconds - so action:", self, 
                "will fail."
            self.status = FAILED
            return
        self.msgDict = {'x': lastLocReport[0].x, 'y': lastLocReport[0].y, 
        'z': lastLocReport[0].z, 'midcatime': self.startTime, 'cmd_id': self.msgID}
        print "trying to send"
        sent = rosrun.send_msg(self.topic, rosrun.dict_as_msg(self.msgDict))
        if not sent:
            if verbose >= 1:
#                 print "Unable to send msg; ", msg, "on topic", topic, " Action", self,  
#                 "assumed failed."
                print "fail!!!!!"
            self.status = FAILED
    
    def check_confirmation(self):
        checkTime = self.lastCheck
        self.lastCheck = midcatime.now()
        feedback = self.mem.get(self.mem.FEEDBACK)
        if not feedback:
            return False
        for item in reversed(feedback):
            #if all items have been checked, either in this check or previous, return
            if item['received_at'] - checkTime < 0:
                return False
            #else see if item is a completion or failure message with id == self.msgID
            if item[CMD_ID_KEY] == self.msgID:
                if item[FEEDBACK_KEY] == COMPLETE:
                    return True
                elif item[FEEDBACK_KEY] == FAILED:
                    self.status = FAILED
                    if verbose >= 1:
                        print "MIDCA received feedback that action", self, "has failed"
                    return False
        return False




class DoReach(AsynchAction):
    
    '''
    Action that orders a point action. To 
    ensure success, an AwaitCurrentLocation action with <= the same maxAllowedLag and the 
    same target should be done immediately before this.
    '''
    
    def __init__(self, mem, midcaAction, objectOrID, maxAllowedLag, maxDuration, topic,
    msgID):
        self.objectOrID = objectOrID
        self.maxAllowedLag = maxAllowedLag
        self.maxDuration = maxDuration
        self.lastCheck = 0.0
        self.topic = topic
        self.complete = False
        self.msgID = msgID
        executeAction = lambda mem, midcaAction, status: self.send_point()
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation()
        AsynchAction.__init__(self, mem, midcaAction, executeAction, 
        completionCheck, True)
    
    
        
    def send_point(self):
        lastLocReport = get_last_location(self.mem, self.objectOrID)
        print lastLocReport
        t = time.now()
        if not lastLocReport:
            if verbose >= 1:
                print "No object location found, so action:", self, "will fail."
            self.status = FAILED
            return
#         if t - lastLocReport[1] > self.maxAllowedLag:
#             if verbose >= 1:
#                 print "Last object location report is too old -", 
#                 (t - lastLocReport[1]).total_seconds(), "seconds - so action:", self, 
#                 "will fail."
#             self.status = FAILED
#             return
        
        x = lastLocReport[0].x
        y = lastLocReport[0].y
        z = lastLocReport[0].z
        
 
                       
        self.msgDict = {'x': x, 'y': y, 
        'z': z, 'time': self.startTime, 'cmd_id': self.msgID}
        
        print self.msgDict
        
        print "trying to send"
        print self.topic
        
        sent = rosrun.send_msg(self.topic, rosrun.dict_as_msg(self.msgDict))
        if not sent:
            if verbose >= 1:
                print "Fail"
#                 print "Unable to send msg; ", msg, "on topic", topic, " Action", self,  
#                 "assumed failed."
            self.status = FAILED
    
    def check_confirmation(self):
        checkTime = self.lastCheck
        self.lastCheck = time.now()
        feedback = self.mem.get(self.mem.FEEDBACK)
        if not feedback:
            return False
        for item in reversed(feedback):
            #if all items have been checked, either in this check or previous, return
            if item['received_at'] - checkTime < 0:
                return False
            #else see if item is a completion or failure message with id == self.msgID
            if item[CMD_ID_KEY] == self.msgID:
                if item[FEEDBACK_KEY] == COMPLETE:
                    return True
                elif item[FEEDBACK_KEY] == FAILED:
                    self.status = FAILED
                    if verbose >= 1:
                        print "MIDCA received feedback that action", self, "has failed"
                    return False
        return False

class DoUnstack(AsynchAction):
    
    '''
    Action that orders a point action. To 
    ensure success, an AwaitCurrentLocation action with <= the same maxAllowedLag and the 
    same target should be done immediately before this.
    '''
    
    def __init__(self, mem, midcaAction, objectOrID, maxAllowedLag, maxDuration, topic,
    msgID):
        self.objectOrID = objectOrID
        self.maxAllowedLag = maxAllowedLag
        self.maxDuration = maxDuration
        self.lastCheck = 0.0
        self.topic = topic
        self.complete = False
        self.msgID = msgID
        self.midcaAction = midcaAction
        executeAction = lambda mem, midcaAction, status: self.send_point()
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation()
        AsynchAction.__init__(self, mem, midcaAction, executeAction, 
        completionCheck, True)
    
    
        
    def send_point(self):
        #we reach the block using the position of the block
        lastLocReport = get_last_location(self.mem, self.objectOrID)
        t = time.now()
        if not lastLocReport:
            if verbose >= 1:
                print "No object location found, so action:", self, "will fail."
            self.status = FAILED
            return
        
        
        x = lastLocReport[0].x
        y = lastLocReport[0].y
        z = lastLocReport[0].z
        #z = 0.02477944410983878

        
        self.msgDict = {'x': x, 'y': y, 
        'z': z, 'time': self.startTime, 'cmd_id': self.msgID}
        
        print self.msgDict
        
        print "trying to send"
        print self.topic
        
        sent = rosrun.send_msg(self.topic, rosrun.dict_as_msg(self.msgDict))
        if not sent:
            if verbose >= 1:
                print "Fail"
#                 print "Unable to send msg; ", msg, "on topic", topic, " Action", self,  
#                 "assumed failed."
            self.status = FAILED
    
    def check_confirmation(self):
        checkTime = self.lastCheck
        self.lastCheck = time.now()
        feedback = self.mem.get(self.mem.FEEDBACK)
        if not feedback:
            return False
        for item in reversed(feedback):
            #if all items have been checked, either in this check or previous, return
            if item['received_at'] - checkTime < 0:
                return False
            #else see if item is a completion or failure message with id == self.msgID
            if item[CMD_ID_KEY] == self.msgID:
                if item[FEEDBACK_KEY] == COMPLETE:
                    return True
                elif item[FEEDBACK_KEY] == FAILED:
                    self.status = FAILED
                    if verbose >= 1:
                        print "MIDCA received feedback that action", self, "has failed"
                    return False
        return False    

class DoStack(AsynchAction):
    
    '''
    Action that orders a point action. To 
    ensure success, an AwaitCurrentLocation action with <= the same maxAllowedLag and the 
    same target should be done immediately before this.
    '''
    
    def __init__(self, mem, midcaAction, objectOrID, maxAllowedLag, maxDuration, topic,
    msgID):
        self.objectOrID = objectOrID
        self.maxAllowedLag = maxAllowedLag
        self.maxDuration = maxDuration
        self.lastCheck = 0.0
        self.topic = topic
        self.complete = False
        self.msgID = msgID
        self.midcaAction = midcaAction
        executeAction = lambda mem, midcaAction, status: self.send_point()
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation()
        AsynchAction.__init__(self, mem, midcaAction, executeAction, 
        completionCheck, True)
    
    
        
    def send_point(self):
        #we need to find the target block?
        lastLocReport = get_last_location(self.mem, self.midcaAction[2])
        t = time.now()
        if not lastLocReport:
            if verbose >= 1:
                print "No object location found, so action:", self, "will fail."
            self.status = FAILED
            return
        if t - lastLocReport[1] > self.maxAllowedLag:
            if verbose >= 1:
                print "Last object location report is too old -", 
                str((t - lastLocReport[1]).total_seconds()), "seconds - so action:", self, 
                "will fail."
            self.status = FAILED
            return
        x = lastLocReport[0].x
        y = lastLocReport[0].y
        z = lastLocReport[0].Z
        
        self.msgDict = {'x': x, 'y': y, 
        'z': z, 'time': self.startTime, 'cmd_id': self.msgID}
        
        print self.msgDict
        
        print "trying to send"
        print self.topic
        
        sent = rosrun.send_msg(self.topic, rosrun.dict_as_msg(self.msgDict))
        if not sent:
            if verbose >= 1:
                print "Fail"
#                 print "Unable to send msg; ", msg, "on topic", topic, " Action", self,  
#                 "assumed failed."
            self.status = FAILED
    
    def check_confirmation(self):
        checkTime = self.lastCheck
        self.lastCheck = time.now()
        feedback = self.mem.get(self.mem.FEEDBACK)
        if not feedback:
            return False
        for item in reversed(feedback):
            #if all items have been checked, either in this check or previous, return
            if item['received_at'] - checkTime < 0:
                return False
            #else see if item is a completion or failure message with id == self.msgID
            if item[CMD_ID_KEY] == self.msgID:
                if item[FEEDBACK_KEY] == COMPLETE:
                    return True
                elif item[FEEDBACK_KEY] == FAILED:
                    self.status = FAILED
                    if verbose >= 1:
                        print "MIDCA received feedback that action", self, "has failed"
                    return False
        return False    


class DoPut(AsynchAction):
    
    '''
    Action that orders a point action. To 
    ensure success, an AwaitCurrentLocation action with <= the same maxAllowedLag and the 
    same target should be done immediately before this.
    '''
    
    def __init__(self, mem, midcaAction, objectOrID, maxAllowedLag, maxDuration, topic,
    msgID):
        self.objectOrID = objectOrID
        self.maxAllowedLag = maxAllowedLag
        self.maxDuration = maxDuration
        self.lastCheck = 0.0
        self.topic = topic
        self.complete = False
        self.msgID = msgID
        executeAction = lambda mem, midcaAction, status: self.send_point()
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation()
        AsynchAction.__init__(self, mem, midcaAction, executeAction, 
        completionCheck, True)
    
    def send_point(self):
        
        point = self.mem.get(self.mem.PUTTING_POINT)

        point1 = copy.deepcopy(point)
        point2 = copy.deepcopy(point1)    

        point1 = check_location_clear(self.mem,point1,1)
        if (point1.x == point2.x) and (point1.y == point2.y) :
            #print("The points are same")
            x = point1.x
            y = point1.y
        else:
            point1 = check_location_clear(self.mem,point1,2)
#            print("The points are different")
#            print(point1)
            x = point1.x
            y = point1.y
        
            
#        print(str(self.mem.get(self.mem.PUTTING_POINT)))
#        raw_input("enter")
        #print("the final point is ")
        #print(point)
        self.msgDict = {'x': x, 'y': y, 
        'z': point.z, 'time': self.startTime, 'cmd_id': self.msgID}
        
        
        sent = rosrun.send_msg(self.topic, rosrun.dict_as_msg(self.msgDict))
        if not sent:
            if verbose >= 1:
                print "Fail"
#                 print "Unable to send msg; ", msg, "on topic", topic, " Action", self,  
#                 "assumed failed."
            self.status = FAILED
        
    
    
    def check_confirmation(self):
        checkTime = self.lastCheck
        self.lastCheck = time.now()
        feedback = self.mem.get(self.mem.FEEDBACK)
        if not feedback:
            return False
        for item in reversed(feedback):
            #if all items have been checked, either in this check or previous, return
            if item['received_at'] - checkTime < 0:
                return False
            #else see if item is a completion or failure message with id == self.msgID
            if item[CMD_ID_KEY] == self.msgID:
                if item[FEEDBACK_KEY] == COMPLETE:
                    return True
                elif item[FEEDBACK_KEY] == FAILED:
                    self.status = FAILED
                    if verbose >= 1:
                        print "MIDCA received feedback that action", self, "has failed"
                    return False
        return False    

class DoRaise(AsynchAction):
    
    '''
    Action that orders a point action. To 
    ensure success, an AwaitCurrentLocation action with <= the same maxAllowedLag and the 
    same target should be done immediately before this.
    '''
    
    def __init__(self, mem, midcaAction, objectOrID, maxAllowedLag, maxDuration, topic,
    msgID):
        self.objectOrID = objectOrID
        self.maxAllowedLag = maxAllowedLag
        self.maxDuration = maxDuration
        self.lastCheck = 0.0
        self.topic = topic
        self.complete = False
        self.msgID = msgID
        executeAction = lambda mem, midcaAction, status: self.send_point()
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation()
        AsynchAction.__init__(self, mem, midcaAction, executeAction, 
        completionCheck, True)
    
    def send_point(self):
#0.6480168766398825, 0.4782503847940384, 0.289534050209461
        raising_point = self.mem.get(self.mem.RAISING_POINT)
        
        self.msgDict = {'x': raising_point.x, 'y': raising_point.y, 
        'z': raising_point.z, 'time': self.startTime, 'cmd_id': self.msgID}

#         self.msgDict = {'x': 0.6480168766398825, 'y': 0.4782503847940384, 
#         'z': 0.289534050209461, 'time': self.startTime, 'cmd_id': self.msgID}
        
        
        sent = rosrun.send_msg(self.topic, rosrun.dict_as_msg(self.msgDict))
        if not sent:
            if verbose >= 1:
                print "Fail"
#                 print "Unable to send msg; ", msg, "on topic", topic, " Action", self,  
#                 "assumed failed."
            self.status = FAILED
    
    def check_confirmation(self):
        checkTime = self.lastCheck
        self.lastCheck = time.now()
        feedback = self.mem.get(self.mem.FEEDBACK)
        if not feedback:
            return False
        for item in reversed(feedback):
            #if all items have been checked, either in this check or previous, return
            if item['received_at'] - checkTime < 0:
                return False
            #else see if item is a completion or failure message with id == self.msgID
            if item[CMD_ID_KEY] == self.msgID:
                if item[FEEDBACK_KEY] == COMPLETE:
                    return True
                elif item[FEEDBACK_KEY] == FAILED:
                    self.status = FAILED
                    if verbose >= 1:
                        print "MIDCA received feedback that action", self, "has failed"
                    return False
        return False
    
class DoGrab(AsynchAction):
    
    '''
    Action that orders a point action. To 
    ensure success, an AwaitCurrentLocation action with <= the same maxAllowedLag and the 
    same target should be done immediately before this.
    '''
    
    def __init__(self, mem, midcaAction, objectOrID, maxAllowedLag, maxDuration, topic,
    msgID):
        self.objectOrID = objectOrID
        self.maxAllowedLag = maxAllowedLag
        self.maxDuration = maxDuration
        self.lastCheck = 0.0
        self.topic = topic
        self.complete = False
        self.msgID = msgID
        executeAction = lambda mem, midcaAction, status: self.send_msg()
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation()
        AsynchAction.__init__(self, mem, midcaAction, executeAction, 
        completionCheck, True)
    
    def send_msg(self):
        self.msgDict = {'cmd': "Grab it", 'time': self.startTime, 'cmd_id': self.msgID}
        
        sent = rosrun.send_msg(self.topic, rosrun.dict_as_msg(self.msgDict))
        if not sent:
            if verbose >= 1:
                print "Fail"
#                 print "Unable to send msg; ", msg, "on topic", topic, " Action", self,  
#                 "assumed failed."
            self.status = FAILED
    
    def check_confirmation(self):
        checkTime = self.lastCheck
        self.lastCheck = midcatime.now()
        feedback = self.mem.get(self.mem.FEEDBACK)
        if not feedback:
            return False
        for item in reversed(feedback):
            #if all items have been checked, either in this check or previous, return
            if item['received_at'] - checkTime < 0:
                return False
            #else see if item is a completion or failure message with id == self.msgID
            if item[CMD_ID_KEY] == self.msgID:
                if item[FEEDBACK_KEY] == COMPLETE:
                    return True
                elif item[FEEDBACK_KEY] == FAILED:
                    self.status = FAILED
                    if verbose >= 1:
                        print "MIDCA received feedback that action", self, "has failed"
                    return False
        return False

class DoRelease(AsynchAction):
    
    '''
    Action that orders a point action. To 
    ensure success, an AwaitCurrentLocation action with <= the same maxAllowedLag and the 
    same target should be done immediately before this.
    '''
    
    def __init__(self, mem, midcaAction, objectOrID, maxAllowedLag, maxDuration, topic,
    msgID):
        self.objectOrID = objectOrID
        self.maxAllowedLag = maxAllowedLag
        self.maxDuration = maxDuration
        self.lastCheck = 0.0
        self.topic = topic
        self.complete = False
        self.msgID = msgID
        executeAction = lambda mem, midcaAction, status: self.send_msg()
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation()
        AsynchAction.__init__(self, mem, midcaAction, executeAction, 
        completionCheck, True)
    
    def send_msg(self):
        self.msgDict = {'cmd': "release it", 'time': self.startTime, 'cmd_id': self.msgID}
        
        sent = rosrun.send_msg(self.topic, rosrun.dict_as_msg(self.msgDict))
        if not sent:
            if verbose >= 1:
                print "Fail"
#                 print "Unable to send msg; ", msg, "on topic", topic, " Action", self,  
#                 "assumed failed."
            self.status = FAILED
    
    def check_confirmation(self):
        checkTime = self.lastCheck
        self.lastCheck = midcatime.now()
        feedback = self.mem.get(self.mem.FEEDBACK)
        if not feedback:
            return False
        for item in reversed(feedback):
            #if all items have been checked, either in this check or previous, return
            if item['received_at'] - checkTime < 0:
                return False
            #else see if item is a completion or failure message with id == self.msgID
            if item[CMD_ID_KEY] == self.msgID:
                if item[FEEDBACK_KEY] == COMPLETE:
                    return True
                elif item[FEEDBACK_KEY] == FAILED:
                    self.status = FAILED
                    if verbose >= 1:
                        print "MIDCA received feedback that action", self, "has failed"
                    return False
        return False

