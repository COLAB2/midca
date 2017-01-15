import os # used to detect operating system for unix-style color codes


class Argument:
	
	'''
	allows operator args to be checked for type and value; e.g. arg 'armMoveSpeed' must be a number between 0.1 and 3 and represents m/s
	'''
	
	def __init__(self, name, checkFunc = None, type = None, strRep = None):
		'''
		checkFunc: a function taking a value as an input and returning whether that value is valid for this argument
		type: runs before checkFunc on the same value; if the value is not of the correct type (or one of them, if type is a tuple), returns False without calling checkFunc
		strRep: replaces the __str__ function; designed to allow custom output that may be more useful.
		'''
		self.name = name
		self.check = checkFunc
		self.type = type
		if strRep:
			self.__str__ = strRep
	
	def valid(self, value):
		if self.type and not isinstance(value, self.type):
			return False
		if self.check:
			return self.check(value)
		return True #no checkFunc; assume valid
	
	#default implementation
	def __str__(self):
		return self.name


class Operator:
	
	'''
	Defines a type of action that an agent can take. Note that this is intended only to define the types of arguments that are valid; the actual functioning to the action should be defined in the execute and/or simulator components. For example, the default symbolic world MIDCA system simply saves a selected action in memory during execute and runs it according to the domain description during simulate. 
	'''
	
	def __init__(self, name, args):
		if not isinstance(name, basestring):
			raise ValueError("Operator name must be a string, got " + str(type(name)))
		self.name = name
		self.args = []
		for arg in args:
			if isinstance(arg, basestring):
				self.args.append(Argument(arg))
			elif isinstance(arg, Argument):
				self.args.append(arg)
			else:
				raise ValueError("Operator args must be strings or MIDCA Argument objects")

class Action:
	def __init__(self, op, *args, **kwargs):
		self.op = op
		self.args = args
	
	def __getitem__(self, item):
		return self.args[item]
	
	def __str__(self):
		s = str(self.op) + "("
		for arg in self.args:
			s += str(arg) + ", "
		s = s[:-2] + ")"
		return s
		
class Action_Old:
	
	def __init__(self, op, *args, **kwargs):
		self.op = op
		self.args = [None for i in range(len(op.args))]
		for i in range(len(args)):
			if op.args[i].valid(args[i]):
				self.args[i] = args[i]
			else:
				raise ValueError(str(args[i]) + " is not a valid value for " + str(op.args[i]))
		if kwargs:
			for i in range(len(op.args)):
				if op.args[i].name in kwargs:
					key = op.args[i].name
					value = kwargs[key]
					if i < len(args):
						raise ValueError("Keyword arg " + key + " has already been assigned to value " + str(args[i]) + ", trying to assign it to " + str(value))
					else:
						if op.args[i].valid(value):
							self.args[i] = value
						else:
							raise ValueError(str(value) + " is not a valid value for " + str(op.args[i]))
	
	#note: if
	def __getitem__(self, item):
		try:
			return self.args[item]
		except ValueError:
			for i in range(len(self.args)):
				if self.op.args[i].name == item:
					return self.args[i]
		raise KeyError(str(item) + " is neither an index nor the name of an argument")
	
	def __str__(self):
		s = self.op.name + "("
		for arg in self.args:
			s += str(arg) + ", "
		s = s[:-2] + ")"
		return s

class Plan:
	
	def __init__(self, actions, goals):
		self.actions = actions
		self.step = 1
		self.goals = goals
	
	def same_plan(self, other):
		return type(self) == type(other) and self.actions == other.actions
	
	def get_next_step(self):
		if len(self.actions) < self.step:
			return None
		return self.actions[self.step - 1]
	
	def advance(self):
		self.step += 1
	
	def get_remaining_steps(self):
		return self.actions[self.step - 1:]
	
	def finished(self):
		return self.step > len(self)
	
	def __len__(self):
		return len(self.actions)
	
	def __getitem__(self, index):
		return self.actions[index]
	
	def last_step_str(self):
		s = ""
		for i in range(len(self.actions)):
			if self.step - 2 == i:
				if os.name == 'nt': # we're on windows, don't use color codes
					s += '[' + str(self.actions[i]) + ']'
				else:
					s += '\033[94m' + str(self.actions[i]) + '\033]0m'
					
			else:
				s += str(self.actions[i])
			s += " "
		return s[:-1]
	
	def __str__(self):
		s = ""
		for i in range(len(self.actions)):
			if self.step - 1 == i:
				if os.name == 'nt': # we're on windows, don't use color codes
					s += '[' + str(self.actions[i]) + ']'
				else:
					s += '\033[94m' + str(self.actions[i]) + '\033]0m'
			else:
				s += str(self.actions[i])
			s += " "
		return s[:-1]
