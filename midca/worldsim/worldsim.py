import copy
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
		self.hash = hash(predicate.name + str(map(str,args))) # little expensive because of map, but only
															  # happens at initialization
		
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
	
	def __hash__(self):
		return self.hash
	
	def __eq__(self, other):
		return self.hash == other.hash
	
	def __ne__(self, other):
		return self.hash != other.hash
	
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
	
	def set_args(self, args):
		'''
		TODO - make this an arg in the constructor
		'''
		self.args = args
		
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
		result_action = Action(self, preconditions, self.prePos, results, self.postPos)
		result_action.set_args(args)
		return result_action
	
	def __str__(self):
		s = "Operator - " + self.name + "("
		for name in self.objnames:
			s += name + ", "
		if self.objnames:
			s = s[:-2]
		s += ")\nPreconditions: ["
		i = 0
		for condition in self.preconditions:
			#print "precondition is "+str(condition)
			if not self.prePos[i]:
				s += "Not "
			s += str(condition) + " ; "
			i += 1
		if self.preconditions:
			s = s[:-3]
		s += "]\nPostconditions: ["
		i = 0
		for condition in self.results:
			#print "postcondition is "+str(condition)
			if not self.postPos[i]:
				s += "Not "
			s += str(condition) + " ; "
			i += 1
		if self.results:
			s = s[:-3] + "]"
		return s



class Tree:
	'''
	Class heirarchy tree which contains insert and print functions
	'''

	def __init__(self,rootnode,allnodes,checked,args):
		'''
		initialize the rootnode , allnodes and checked .
		rootnode is the main node that contains the trace of all nodes.
		checked is something which is usefull in goaltransforms , 
		to check whether the result is not as same as what to be transformed.
		'''
                self.rootnode = rootnode
                self.allnodes = allnodes
                self.checked = checked
		if not args == 0:
			self.insert(args)

        def printall(self,space,printed,root):
		'''
		Print the predicates in the heirarchy of spaces. 
		for example rootnode should contain no space while the following children 
		should contain a tab space and so on.
		'''
		if (len(root.parents) == 0) and (not( len(printed) == 0)):
			return 0
		if not root.predicate in printed:
			printed.append(root.predicate)
			print(space + root.predicate)
			space =  space[0:4] + space
		else:
			space = space[0:-4]
		if len(root.children) == 0:
			self.printall(space, printed,root.parents.pop())
		else:
			for s in root.children:
				self.printall(space, printed,s)

        def printtree(self):
		'''
		Start from the root node and print all the predicates by parsing through the children.
		'''
                root = copy.deepcopy(self.rootnode)
                printed = list()
                space = "  "
                print("--------- Class Hierarchy Predicate Tree ----------")
                self.printall(space,printed,root)
                print("")
                print("--------- Class Hierarchy Predicate Tree Ends ----------")
                print("")

	def insert(self,content):
		'''
		create a node for each new argument and call it predicate , 
		if there is no root node initialize root node 
		else check the node to add a branch for and add a branch to it.
		'''
	     	for i in range(0,len(content)):
			newnode = Node()
			temp_string = content
			newnode = newnode.insert(temp_string[len(temp_string)-1])
		
		if not(self.rootnode):
                 	 self.rootnode= newnode
                 	 self.allnodes.append(newnode)
		
		for i in range(0,len(temp_string)-1):
			newnode = Node()
			newnode = newnode.insert(temp_string[i])
			newnode.insertbranch(self.check_in_all_tree_nodes(temp_string[len(temp_string)-1]),self)
			self.allnodes.append(newnode)

	def check_in_all_tree_nodes(self,predicate):
		for a in range(0,len(self.allnodes)):
			if self.allnodes[a].predicate == predicate:
				return self.allnodes[a]


class ObjectTree:

	'''
	Class heirarchy tree which contains insert and print functions
	'''

	def __init__(self,rootnode,allnodes,checked,args):
                self.rootnode = rootnode
                self.allnodes = allnodes
                self.checked = checked
		if not args == 0:
			self.objectinsert(args)

        def printall(self,space,printed,root):
		'''
		Print the predicates in the heirarchy of spaces. for example rootnode 		
		should contain no space while the following children 			
		should contain a tab space and so on.
		'''
	
		if (len(root.parents) == 0) and (not( len(printed) == 0)):
			return 0

		if not root.predicate in printed:
			printed.append(root.predicate)
			print(space + root.predicate)
			space =  space[0:4] + space
		else:
				
			space = space[0:-4]

		if len(root.children) == 0:
			self.printall(space, printed,root.parents.pop())
		else:
			for s in root.children:
				self.printall(space, printed,s)

        
        def printtree(self):
		'''
		Start from the root node and print all the predicates by parsing through the children.
		'''
                root = copy.deepcopy(self.rootnode)
                printed = list()
                space = "  "
                print("--------- Class Hierarchy Object Tree ----------")
                self.printall(space,printed,root)
                print("")
                print("--------- Class Hierarchy Object Tree Ends ----------")
                print("")

	def objectinsert(self,content):
		'''
		create a node for each new argument and call it predicate , 
		if there is no root node initialize root node 
		else check the node to add a branch for and add a branch to it.
		'''
	   	for i in range(0,len(content)):
			temp_string = content
			newnode = Node()
			newnode = newnode.insert(temp_string[len(temp_string)-1])
		
		if not(self.rootnode):
                 	 self.rootnode= newnode
                 	 self.allnodes.append(newnode)
		
		for i in range(0,len(temp_string)-1):
			newnode = Node()
			newnode = newnode.insert(temp_string[i])
			newnode.insertbranch(self.check_in_all_object_nodes(temp_string[len(temp_string)-1]),self)
			self.allnodes.append(newnode)
		
	def check_in_all_object_nodes(self,predicate):
		for a in range(0,len(self.allnodes)):
			if self.allnodes[a].predicate == predicate:	
				return self.allnodes[a]	

class Node:
	'''
	Node for creating a tree which contains predicate as name , 
	parents and children as data members.
	'''
        def __init__(self):
                self.predicate = ""
                self.parents = list()
                self.children = list()

        def insert(self,predicate):
                self.predicate = predicate
                return self
                       
                                       
        def insertbranch(self,s,tree):
                self.parents.append(s)
                s.children.append(self)
                for a in range(0,len(tree.allnodes)):
                        if tree.allnodes[a].predicate is s.predicate:
                                tree.allnodes[a] = s                         
                return self

		
class World:
	
	def __init__(self,operators, predicates, atoms, types, objects = [],cltree = [] , obtree = []):
		self.operators = {}
		self.types = types
		self.cltree = cltree
		self.obtree = obtree
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
		self.atoms = set(atoms)
	
	def get_atoms(self,filters=[]):
		'''
		Will return atoms if the filter (string) is within any pred or arg names
		If more than one filter string, then all must match (conjunctive filters)
		'''
		
		# if no filters, return everything
		if len(filters) == 0:
			return self.atoms
		
		# record which filters have matched
		filter_matches = {k:False for k in filters}
		#print("filter matches = "+str(filter_matches))
		relevant_atoms = []
		if len(filters) > 0:
			for atom in self.atoms:
				atom_parts = [atom.predicate]+atom.args
				for filter_str in filters: # assuming there shouldn't be more than 3-4 filters
					for part in atom_parts: # assuming shouldn't be atoms with more than 4-5 parts
						#print "type(part) == "+str(part.name)
						if (not filter_matches[filter_str]) and filter_str in part.name:
							filter_matches[filter_str] = True
							
				if not (False in filter_matches.values()): # check to see they are all True
					relevant_atoms.append(atom)
					#print("Just added "+str(atom)+" to relevant atoms")
				# reset filter matches
				filter_matches = {k:False for k in filters}
				
		return relevant_atoms

	def get_objects_names_by_type(self, typename):
		objs = []
		for each_obj in self.objects.keys():
			if(str(self.obj_type(each_obj)) == str(typename)):
				objs.append(each_obj)
		return objs
	
	def diff(self,otherWorld):
		'''
		Given another world, return the differences in atoms.
		Return value is a tuple where the first element is the atoms in this world not in the given world
		Second element is the atoms not in this world but in given world
		'''
		atoms_not_in_other = []
		atoms_not_in_self = []
		# go through all atoms in this world, check to see if they are in other world
		for atom in self.atoms:
			if not otherWorld.atom_true(atom):
				atoms_not_in_other.append(atom)
				
		# go through all atoms in given world, check to see if they are in this world
		for atom in otherWorld.atoms:
			if not self.atom_true(atom):
				atoms_not_in_self.append(atom)
		
		return (atoms_not_in_self,atoms_not_in_other)
	
	def equal(self,otherWorld):
		diff_result = self.diff(otherWorld)
		#print "diff_result inside equal() : "+str(map(str,diff_result[0]))+","+str(map(str,diff_result[1]))
		return  diff_result == ([],[])
	
	def fast_equal(self,otherworld):
		pass
	
	def copy(self):
		# safety check, in case a list is passed for atoms instead of a set
		if not type(self.atoms) is set:
			self.atoms = set(self.atoms)
			
		return World(self.operators.values(), self.predicates.values(), self.atoms.copy(), self.types.copy(), self.objects.values())
	
	def is_true(self, predname, argnames = []):
		for atom in self.atoms:
			if atom.predicate.name == predname:
				if len(atom.args) == len(argnames):
					namesCorrect = True
					for i in range(len(atom.args)):
						if atom.args[i].name != argnames[i]:
							namesCorrect = False
					if namesCorrect:
						return True
		return False
	
	def atom_true(self, atom):
		# this is very fast, because atom objects have hashes and self.atoms is a set, not a list
		return atom in self.atoms
	
	def add_atom(self, atom):
		self.atoms.add(atom)
	
	def add_fact(self, predname, argnames = []):
		if not self.is_true(predname, argnames):
			self.add_atom(Atom(self.predicates[predname], [self.objects[name] for name in argnames]))
	
	def remove_atom(self, atom):
		self.atoms.remove(atom)
				
	def remove_fact(self, predname, argnames = []):
		toRemove = None
		for atom in self.atoms:
			if atom.predicate.name == predname:
				if len(atom.args) == len(argnames):
					namesCorrect = True
					for i in range(len(atom.args)):
						if atom.args[i].name != argnames[i]:
							namesCorrect = False
					if namesCorrect:
						toRemove = atom
						break
		if toRemove:
			self.remove_atom(toRemove)
	
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
			actualObject = self.objects[object]
			del(self.objects[object])
			self.atoms = [atom for atom in self.atoms if actualObject not in atom.args]
			return True
		else:
			for key in self.objects:
				if self.objects[key] == object:
					del(self.objects[key])
					self.atoms = [atom for atom in self.atoms if object not in atom.args]
					return True
		return False
	
	def get_possible_objects(self, predicate, arg):
		return self.objects.values() #not, obviously, a good implementation
	
	def get_objects_by_type(self, some_type):
		if type(some_type) is str:
			if some_type not in self.get_types():
				raise Exception("Trying to get object of type "+str(some_type)+" but not a valid type")
		else:
			if some_type.name not in self.get_types():
				raise Exception("Trying to get object of type "+str(some_type)+" but not a valid type")
			
		objs = []
		for obj in self.objects.values():
			if obj.is_a(some_type):
				objs.append(obj)
		
		return objs
	
	def get_types(self):
		return self.types
	
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
			operator = self.operators[midcaAction.op]
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
				#print("removing_atom "+str(simAction.results[i]))
				self.remove_atom(simAction.results[i])
	
	def apply_named_action(self, opName, argNames):
		args = []
		for name in argNames:
			if name not in self.objects:
				raise Exception("Object " + name + " DNE")
			args.append(self.objects[name])
		if opName not in self.operators:
			raise Exception("Operator " + opName + " DNE")
		simAction = self.operators[opName].instantiate(args)
		if not self.is_applicable(simAction):
			raise Exception("Preconditions not met.")
		self.apply(simAction)
	
	#convenience method for operating with MIDCA
	def apply_midca_action(self, midcaAction):
		opname = midcaAction.op
		argnames = [str(arg) for arg in midcaAction.args]
		self.apply_named_action(opname, argnames)
	
	#interprets a MIDCA goal as a predicate statement. Expects the predicate name to be either in kwargs under 'predicate' or 'Predicate', or in args[0]. This is complicated mainly due to error handling.
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
		return True
	
	def goals_achieved(self, plan, goalSet):
		testWorld = self.copy()
		achievedGoals = set()
		for action in plan.get_remaining_steps():
			if not testWorld.midca_action_applicable(action):
				break
			testWorld.apply_midca_action(action)
		for goal in goalSet:
			achieved = testWorld.atom_true(self.midcaGoalAsAtom(goal))
			if 'negate' in goal and goal['negate']:
				achieved = not achieved
			if achieved:
				achievedGoals.add(goal)
		return achievedGoals
	
	def goals_achieved_now(self, goalSet):
		'''
		Same as goals_achieved but no plan, uses this world
		'''
		for goal in goalSet:
			achieved = self.atom_true(self.midcaGoalAsAtom(goal))
			if 'negate' in goal and goal['negate']:
				achieved = not achieved
			if not achieved:
				return False
			
		return True
	
	def plan_goals_achieved(self, plan):
		return self.goals_achieved(plan, plan.goals)

	def get_operators(self):
		return self.operators
	
	def remove_operator(self, opname):
		if opname in self.operators.keys():
			del self.operators[opname]
			return True
		return False
	
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

