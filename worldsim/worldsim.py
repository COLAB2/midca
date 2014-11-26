class Obj:
	
	def __init__(self, name, type):
		self.name = name
		self.type = type
	
	def is_a(self, type):
		return self.type.is_a_(type)
	
	def ancestors(self):
		return self.type.ancestors()
	
	def __str__(self):
		return self.name

class Type:
	
	def __init__(self, name, parents = []):
		self.parents = parents
		self.name = name
	
	#optimize (i.e. save ancestor set) if necessary
	def ancestors(self):
		ancestors = {}
		for parent in self.parents:
			ancestors[parent] = True
			for ancestor in parent.ancestors():
				ancestors[ancestor] = True
		return ancestors

	def is_a_(self, type):
		return type == self or type in self.ancestors()
	
	def instantiate(self, name):
		return Obj(name, self)
	
	def __str__(self):
		return "Type: " + self.name

class Atom:
	
	def __init__(self, predicate, args):
		if len(predicate.argnames) != len(args):
			raise Exception("Wrong number of args for " + predicate.name)
		i = 0
		if predicate.argtypes:
			for arg in args:
				if not arg.is_a(predicate.argtypes[i]):
					raise Exception("Instantiating argument " + predicate.argnames[i] + " with " + arg.name + ", which is the wrong type of object")
				i += 1
		self.predicate = predicate
		self.args = args
	
	def __getitem__(self, item):
		if item in self.predicate.argnames:
			return self.args[self.predicate.argnames.index(item)]
		try:
			return self.args[item]
		except Exception:
			raise Exception("value must be name or index of argument")
	
	def __str__(self):
		s = self.predicate.name + "("
		for arg in self.args:
			s += arg.name + ", "
		if self.args:
			s = s[:-2]
		return s + ")"
	
	def __eq__(self, other):
		if self.predicate != other.predicate:
			return False
		for i in range(len(self.args)):
			if self[i] != other[i]:
				return False
		return True

class Predicate:
	
	def __init__(self, name, argnames, argtypes = []):
		if argtypes and len(argnames) != len(argtypes):
			raise Exception("argnames list must be same length as argtypes")
		self.name = name
		self.argnames = argnames
		self.argtypes = argtypes
	
	def instantiate(self, args):
		return Atom(self, args)
	
	def __str__(self):
		s = "Predicate: " + self.name + "("
		i = 0
		for arg in self.argnames:
			s += arg
			if self.argtypes:
				s += "[" + self.argtypes[i].name + "]"
			s += ", "
			i += 1
		if self.argnames:
			s = s[:-2]
		return s + ")"

class Action:
	
	def __init__(self, operator, preconds, prePos, results, postPos):
		self.operator = operator
		self.preconds = preconds
		self.prePos = prePos
		self.results = results
		self.postPos = postPos
	
	def __str__(self):
		s = "Action - " + self.operator.name + ":\nPreconditions: ["
		i = 0
		for condition in self.preconds:
			if not self.prePos[i]:
				s += "Not "
			s += str(condition) + " ; "
			i += 1
		if self.preconds:
			s = s[:-3]
		s += "]\nResults: ["
		i = 0
		for condition in self.results:
			if not self.postPos[i]:
				s += "Not "
			s += str(condition) + " ; "
			i += 1
		if self.results:
			s = s[:-3] + "]"
		return s

class Condition:
	
	def __init__(self, atom, argtypes):
		self.atom = atom
		self.argtypes = argtypes
	
	def instantiate(self, args):
		if self.argtypes:
			i = 0
			for arg in args:
				if not arg.is_a(self.argtypes[i]):
					raise Exception("Trying to instantiate " + arg.name + " as a " + self.argtypes[i].name)
				i += 1
		return self.atom.predicate.instantiate(args)
	
	def fits(self, args):
		if self.argtypes:
			i = 0
			for arg in args:
				if not arg.is_a(self.argtypes[i]):
					return False
				i += 1
		if self.atom.predicate.argtypes:
			i = 0
			for arg in args:
				if not arg.is_a(self.atom.predicate.argtypes[i]):
					return False
				i += 1
		for i in range(len(self.atom.args)):
			for i2 in range(i + 1, len(self.atom.args)):
				if self.atom[i] == self.atom[i2] and args[i] != args[i2]:
					return False
		return True
	
	def __str__(self):
		return str(self.atom)

class Operator:
	
	def __init__(self, name, objnames, prepredicates, preobjnames, preobjtypes, prePositive, postpredicates, postobjnames, postobjtypes, postPositive):
		self.name = name
		self.objnames = objnames
		self.preconditions = {}
		self.precondorder = []
		self.prePos = prePositive
		for pred in range(len(prepredicates)):
			args = []
			usednames = []
			names = preobjnames[pred]
			types = preobjtypes[pred]
			for arg in range(len(names)):
				if names[arg] in usednames:
					args.append(args[names.index(names[arg])])
				else:
					args.append(types[arg].instantiate(names[arg]))
				usednames.append(names[arg])
			cond = Condition(prepredicates[pred].instantiate(args), types)
			self.precondorder.append(cond)
			self.preconditions[cond] = names
		self.results = {}
		self.resultorder = []
		self.postPos = postPositive
		for pred in range(len(postpredicates)):
			args = []
			usednames = []
			names = postobjnames[pred]
			types = postobjtypes[pred]
			for arg in range(len(names)):
				if names[arg] in usednames:
					args.append(args[names.index(names[arg])])
				else:
					args.append(types[arg].instantiate(names[arg]))
				usednames.append(names[arg])
			cond = Condition(postpredicates[pred].instantiate(args), types)
			self.resultorder.append(cond)
			self.results[cond] = names
	
	def instantiate(self, args):
		if len(args) != len(self.objnames):
			raise Exception("wrong number of arguments")
		objdict = {}
		for i in range(len(args)):
			objdict[self.objnames[i]] = args[i]
		preconditions = []
		for condition in self.precondorder:
			names = self.preconditions[condition]
			args = []
			for name in names:
				args.append(objdict[name])
			preconditions.append(condition.instantiate(args))
		results = []
		for condition in self.resultorder:
			names = self.results[condition]
			args = []
			for name in names:
				args.append(objdict[name])
			results.append(condition.instantiate(args))
		return Action(self, preconditions, self.prePos, results, self.postPos)
	
	def __str__(self):
		s = "Operator - " + self.name + "("
		for name in self.objnames:
			s += name + ", "
		if self.objnames:
			s = s[:-2]
		s += ")\nPreconditions: ["
		i = 0
		for condition in self.preconditions:
			if not self.prePos[i]:
				s += "Not "
			s += str(condition) + " ; "
			i += 1
		if self.preconditions:
			s = s[:-3]
		s += "]\nPostconditions: ["
		i = 0
		for condition in self.results:
			if not self.postPos[i]:
				s += "Not "
			s += str(condition) + " ; "
			i += 1
		if self.results:
			s = s[:-3] + "]"
		return s
		
class World:
	
	def __init__(self, operators, predicates, atoms, types, objects = []):
		self.operators = {}
		self.types = types
		for operator in operators:
			self.operators[operator.name] = operator
		self.predicates = {}
		for predicate in predicates:
			self.predicates[predicate.name] = predicate
		self.objects = {}
		for atom in atoms:
			for arg in atom.args:
				self.objects[arg.name] = arg
		for object in objects:
			self.objects[object.name] = object
		self.atoms = atoms
	
	def copy(self):
		return World(self.operators.values(), self.predicates.values(), self.atoms[:], self.types.copy(), self.objects.values())
	
	def is_true(self, predname, argnames):
		try:
			args = [self.objects[name] for name in argnames]
			return self.atom_true(Atom(self.predicates[predname], args))
		except Exception:
			return False
	
	def atom_true(self, atom):
		return atom in self.atoms
	
	def add_atom(self, atom):
		if atom not in self.atoms:
			self.atoms.append(atom)
	
	def add_fact(self, predname, argnames):
		self.add_atom(Atom(self.predicates[predname], [self.objects[name] for name in argnames]))
	
	def remove_atom(self, atom):
		if atom in self.atoms:
			self.atoms.remove(atom)
	
	def add_object(self, object):
		self.objects[object.name] = object
	
	def add_object_by_type(self, name, type):
		if type in self.types:
			self.add_object(self.types[type].instantiate(name))
		else:
			raise Exception("Type " + str(type) + " DNE.")
	
	def obj_type(self, objname):
		return self.objects[objname].type.name
	
	def remove_object(self, object):
		if object in self.objects:
			del(self.objects[object])
		else:
			for key in self.objects:
				if self.objects[key] == object:
					del(self.objects[key])
					break
	
	def get_possible_objects(predicate, arg):
		return self.objects.values() #not, obviously, a good implementation
	
	def is_applicable(self, action):
		for i in range(len(action.preconds)):
			if action.prePos[i] and not self.atom_true(action.preconds[i]):
				return False
			if not action.prePos[i] and self.atom_true(action.preconds[i]):
				return False
		return True
	
	#convenience method for operating with MIDCA
	def midca_action_applicable(self, midcaAction):
		try:
			operator = self.operators[midcaAction.op.name]
			args = [self.objects[arg] for arg in midcaAction.args]
		except KeyError:
			return False
		action = operator.instantiate(args)
		return self.is_applicable(action)

	def apply(self, simAction):
		for i in range(len(simAction.results)):
			if simAction.postPos[i]:
				self.add_atom(simAction.results[i])
			else:
				self.remove_atom(simAction.results[i])
	
	def apply_named_action(self, opName, argNames):
		args = []
		for name in argNames:
			if name not in self.objects:
				raise Exception("Object " + name + " DNE")
			args.append(self.objects[name])
		if opName not in self.operators:
			raise Exception("Operator " + opname + " DNE")
		simAction = self.operators[opName].instantiate(args)
		if not self.is_applicable(simAction):
			raise Exception("Preconditions not met.")
		self.apply(simAction)
	
	#convenience method for operating with MIDCA
	def apply_midca_action(self, midcaAction):
		opname = midcaAction.op.name
		argnames = [str(arg) for arg in midcaAction.args]
		self.apply_named_action(opname, argnames)
	
	#interprets a MIDCA goal as a predicate statement. Expects the predciate name to be either in kwargs under 'predicate' or 'Predicate', or in args[0]. This is complicated mainly due to error handling.
	def midcaGoalAsAtom(self, goal):
		try:
			predName = str(goal['predicate'])
		except KeyError:
			try:
				predName = str(goal['Predicate'])
			except KeyError:
				try:
					predName = str(goal[0])
				except KeyError:
					raise ValueError("Trying to interpret " + str(goal) + " as a predicate atom, but cannot find a predicate name.")
		try:
			predicate = self.predicates[predName]
		except KeyError:
			raise ValueError("Predicate " + predName + " not in domain.")
		
		args = [] #args for new atom
		#check if predicate took first spot in arg list
		if goal.args[0] != "predicate":
			nextArgI = 0
		else:
			nextArgI = 1
		for i in range(len(predicate.argnames)):
			if nextArgI < len(goal.args):
				try:
					args.append(self.objects[str(goal.args[nextArgI])])
					nextArgI += 1
				except KeyError:
					raise ValueError("Object " + str(goal.args[nextArgI]) + " not found; goal " + str(goal) + " does not encode a valid predicate representation.")
			else:
				if predicate.argnames[i] in goal.kwargs:
					try:
						value = goal.kwargs[predicate.argnames[i]]
						args.append(self.objects[str(value)])
					except KeyError:
						raise ValueError("Object " + str(value) + " not found; goal " + str(goal) + " does not encode a valid predicate representation.")
				else:
					raise ValueError("Trying to interpret " + str(goal) + " as a predicate atom, but cannot find a value for argument " + predicate.argnames[i])
		assert len(args) == len(predicate.argnames) #sanity check
		try:
			return Atom(predicate, args)
		except Exception:
			raise ValueError(str(predicate) + str(args) + " does not seem to be a valid state")
	
	def plan_correct(self, plan):
		testWorld = self.copy()
		for action in plan.get_remaining_steps():
			if not testWorld.midca_action_applicable(action):
				return False
			testWorld.apply_midca_action(action)
		for goal in plan.goals:
			achieved = testWorld.atom_true(self.midcaGoalAsAtom(goal))
			if not achieved:
				return False
		return True

	def __str__(self):
		s = "[\n"
		for name in sorted(self.objects.keys()):
			object = self.objects[name]
			s += object.name + " (" + object.type.name + ") : "
			for atom in self.atoms:
				if object in atom.args:
					s += str(atom) + " and "
			if s[-2:] == "d ":
				s = s[:-5] + "\n"
			else:
				s = s[:-3] + "\n"
		return s + "]\n"

