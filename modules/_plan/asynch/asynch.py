from MIDCA import rosrun, plans
from MIDCA import midcatime
import traceback
try:
        from geometry_msgs.msg import PointStamped
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
				(t - lastLocReport[1]).total_seconds(), "seconds - so action:", self, 
				"will fail."
			self.status = FAILED
			return
		self.msgDict = {'x': lastLocReport[0].x, 'y': lastLocReport[0].y, 
		'z': lastLocReport[0].z, 'midcatime': self.startTime, 'cmd_id': self.msgID}
		print "trying to send"
		sent = rosrun.send_msg(self.topic, rosrun.dict_as_msg(self.msgDict))
		if not sent:
			if verbose >= 1:
				print "Unable to send msg; ", msg, "on topic", topic, " Action", self,  
				"assumed failed."
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
