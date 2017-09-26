from midca.logging import Event
from midca.trace import CogTrace
import threading

class Memory:
	
	'''
	To do: in theory, a lock could be deleted in between when its existence is checked
	and when it is acquired. At some point this needs to be fixed.
	'''
	
	#if necessary for efficiency, these keys should be numbers or enum constants.
	STATES = "__world states"
	STATE = "__current state"
	GOAL_GRAPH = "__goals"
	CURRENT_GOALS = "__current goals"
	PLANS = "__plans"
	ACTIONS = "__actions"
	FEEDBACK = "__feedback"
	#ROS constants used by rosrun classes and related modules in act and perceive.
	ROS_OBJS_DETECTED = "__detected object queue"
	ROS_OBJS_STATE = "__state"
	STATE_HISTORY = "__history"
	CALIBRATION_MATRIX = "__camera calibration status"
	CALIBRATION_Z = "__Z"
	STACK_Z = "__SZ"
	UNSTACK_Z = "__UZ"
	STACK_3Z = "__3SZ"
	UNSTACK_3Z = "__3SZ"
	RAISING_POINT = "__RP"
	PUTTING_POINT = "__PP"
	ROS_WORDS_HEARD = "__words heard queue"
	ROS_FEEDBACK = "__ros feedback"
	Objects = "__objects"
	DELIVERED = "__deliveredscore"
	#MetaCognitive
	TRACE_SEGMENT = "__trace segment"
	META_ANOMALIES = "__meta anomalies"
	META_GOALS = "__meta goals"
	META_CURR_GOAL = "__meta current goal"
	META_PLAN = "__meta plan" #TODO allow more than one plan
	
	# data recording
	PLANNING_COUNT = "__PlanningCount" # number of times planning was performed (including replanning)
	GOALS_ACTIONS_ACHIEVED = "__GoalsActionsAchieved" # number of goals achieved
	GOALS_ACHIEVED = "__GoalsAchieved" # number of goals achieved
	ACTIONS_EXECUTED = "__ActionsExecuted" # number of actions executed
	MIDCA_CYCLES = "__MIDCA Cycles"
	CURR_PLAN = "__CurrPlan"
	
	# GDA
	DISCREPANCY = "__discrepancy"
	EXPLANATION = "__explanation"
	EXPLANATION_VAL = "__explanation_val"


    	#Goal Tranformation
    	CL_TREE = "__class heirarchy tree"
    	OB_TREE = "__object heirarchy tree"

    	#Construction 
    	TIME_CONSTRUCTION = "__Construction Time"
    	ACTUAL_TIME_CONSTRUCTION = "__Actual Time"
    	EXPECTED_TIME_CONSTRUCTION = "__Expected Time"
    	SELECTED_BUILDING_LIST = "__Buildings Selected"
    	COMPLETE_BUILDING_LIST = "__Buildings"
    	EXECUTED_BUILDING_LIST = "__Executed Buildings"
    	REJECTED_GOALS = "__rejected goals"
    	ACTUAL_SCORES ="__Expected Scores"
    	P = "__ list of scores"
    	t = "__list of times"

    	# Restaurant
    	MONEY = "__money limit"
    	SELECTED_ORDERS = "__selected orders"
    	COMPLETED_ORDERS = "__completed orders"
    	EXPECTED_SCORE = "__expected score"
    	ACTUAL_SCORE = "__actual score"
    	EXPECTED_COST= "__expected cost"
    	ACTUAL_COST= "__actual cost"

	def __init__(self, args = {}):
		self.knowledge = {}
		self.update(args)
		self.logger = None
		self.mainLock = threading.Lock() #to synchronize lock creation
		self.locks = {} #lock for each key
		self.logEachAccess = True
		#MetaCognitive Variables
		self.metaEnabled = False
		self.myMidca = None # pointer to MIDCA object
		self.trace = False	

	#Handles structs with custom update methods, dict update by dict or tuple, list append, and simple assignment.
	def _update(self, structname, val):
		with self.mainLock:
			if structname not in self.locks:
				self.locks[structname] = threading.Lock()
		with self.locks[structname]:
			if not structname in self.knowledge:
				self.knowledge[structname] = val
			elif self.knowledge[structname].__class__.__name__ == "dict":
				if val.__class__.__name__ == "dict":
					self.knowledge[structname].update(val) #update dict with dict
				elif len(val) == 2:
					self.knowledge[structname][val[0]] = val[1] #update dict with tuple
			elif hasattr(self.knowledge[structname], "update"):
				self.knowledge[structname].update(val) #generic update
			else:
				self.knowledge[structname] = val #assignment
			self.logAccess(structname)

	def add(self, structname, val):
		'''
		Used to create lists of items. If nothing is stored under structname, will create
		a one-item list containing val. If there is a list, will append val. If some item
		is stored with no append method, will create a two-item list with the previously
		stored item and val.
		'''
		with self.mainLock:
			if structname not in self.locks:
				self.locks[structname] = threading.Lock()
		with self.locks[structname]:
			if not structname in self.knowledge:
				self.knowledge[structname] = [val]
			elif hasattr(self.knowledge[structname], "append"):
				self.knowledge[structname].append(val)
			else:
				self.knowledge[structname] = [self.knowledge[structname], val]
			self.logAccess(structname)

	def set(self, structname, val):
		with self.mainLock:
			if structname not in self.locks:
				self.locks[structname] = threading.Lock()
		with self.locks[structname]:
			self.knowledge[structname] = val
			self.logAccess(structname)

	def update(self, args):
		for structname, val in args.items():
			self._update(structname, val)

	def update_all(self, structname, val):
		with self.mainLock:
			if structname not in self.locks:
				self.locks[structname] = threading.Lock()
		with self.locks[structname]:
			if structname in self.knowledge and (not isinstance(self.knowledge[structname], basestring)):
				struct = self.knowledge[structname]
				if hasattr(struct, "__getitem__") or hasattr(struct, "__iter__"):
					for item in struct:
						if hasattr(item, "update"):
							item.update(val)
				elif hasattr(struct, "update"):
					struct.update(val)
			self.logAccess(structname)

	def remove(self, structname):
		with self.mainLock:
			if structname not in self.locks:
				self.locks[structname] = threading.Lock()
		with self.locks[structname]:
			self.logAccess(structname)
			if structname in self.knowledge:
				del self.knowledge[structname]
				del self.locks[structname]

	def clear(self):
		self.knowledge.clear()
		self.locks.clear()

	def get(self, structname):
		with self.mainLock:
			if structname not in self.locks:
				return None #if there is knowledge stored there must be a lock.
		with self.locks[structname]:
			self.logAccess(structname)
			if structname in self.knowledge:
				return self.knowledge[structname]
			return None

	def get_and_clear(self, structname):
		with self.mainLock:
			if structname not in self.locks:
				return None #if there is knowledge stored there must be a lock.
		with self.locks[structname]:
			self.logAccess(structname)
			val = self.knowledge[structname]
			del self.knowledge[structname]
			del self.locks[structname]
			return val

	def get_and_lock(self, structname):
		with self.mainLock:
			if structname not in self.locks:
				self.locks[structname] = threading.Lock()
				self.locks[structname].acquire()
				return None
		self.locks[structname].acquire()
		self.logAccess(structname)
		return self.knowledge[structname]

	def unlock(self, structname):
		try:
			self.locks[structname].release()
		except KeyError:
			pass

	def enableLogging(self, logger):
		self.logger = logger

	def logAccess(self, key):
		if self.logger and self.logEachAccess:
			self.logger.logEvent(MemAccessEvent(key))

	def enableMeta(self, phaseManager):
		if not self.trace:
			raise Exception("Please call mem.enableTrace() before calling enableMeta()")
		self.metaEnabled = True
		self.myMidca = phaseManager

	def enableTrace(self):
		if not self.trace:
			self.trace = CogTrace()

class MemAccessEvent(Event):

	def __init__(self, keyAccessed):
		self.keys = ['log', 'Memory Access']
		self.loggable = True
		self.memKey = keyAccessed

	def __str__(self):
		return "Memory access at key " + str(self.memKey)
