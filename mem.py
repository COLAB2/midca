
from MIDCA.logging import Event

class Memory:
	
	STATES = "__world states"
	GOAL_GRAPH = "__goals"
	CURRENT_GOALS = "__current goals"
	PLANS = "__plans"
	ACTIONS = "__actions"
	
	def __init__(self, args = {}):
		self.knowledge = {}
		self.clear()
		self.update(args)
		self.logger = None
                self.trace = None
                self.myMidca = None

	#Handles structs with custom update methods, dict update by dict or tuple, list append, and simple assignment.
	def _update(self, structname, val):
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
		if not structname in self.knowledge:
			self.knowledge[structname] = [val]
		elif hasattr(self.knowledge[structname], "append"):
			self.knowledge[structname].append(val)
		else:
			self.knowledge[structname] = [self.knowledge[structname], val]
		self.logAccess(structname)
	
	def set(self, structname, val):
		self.knowledge[structname] = val
		self.logAccess(structname)
	
	def update(self, args):
		for structname, val in args.items():
			self._update(structname, val)
	
	def update_all(self, structname, val):
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
		self.logAccess(structname)
		if structname in self.knowledge:
			del self.knowledge[structname]
	
	def clear(self):
		self.knowledge.clear()
	
	def get(self, structname):
		self.logAccess(structname)
		if structname in self.knowledge:
			return self.knowledge[structname]
		return None
	
	def enableLogging(self, logger):
		self.logger = logger
	
	def logAccess(self, key):
		if self.logger:
			self.logger.logEvent(MemAccessEvent(key))
                        
        def enableTracing(self, trace):
                self.trace = trace

        def setMyMidca(self, myMidca):
                self.myMidca = myMidca
                


class MemAccessEvent(Event):

	def __init__(self, keyAccessed):
		self.keys = ['log', 'Memory Access']
		self.loggable = True
		self.memKey = keyAccessed
	
	def __str__(self):
		return "Memory access at key " + str(self.memKey)
	
	
