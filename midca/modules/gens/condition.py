import cl

class preconditions:
	
	objs = []
	predicates = []
	def __init__(self):
		for item in cl.ObjectTree.allnodes:
			self.objs.append(item.predicate)
		for item in cl.Tree.allnodes:
			self.predicates.append(item.predicate)
		
	def identity_conditions(self):
		return True


	def genral_precondition1(self,p,obj):
		for item in cl.Tree.allnodes:
			if item.predicate == p:
			     for each in obj:
				if not each in preconditions.objs :
					return False
			     return True
		else:
			return False

	def genral_precondition2(self,p):

		for item in cl.Tree.allnodes:
			if item.predicate == p:
				if len(item.parents):
					return item.parents
		return False 

	def genral_precondition3(self):
		return True

	def specail_precondition2(self,p):
		for item in cl.Tree.allnodes:
			if item.predicate == p:
				if len(item.children):
					for a in item.children:
                                               if not a.predicate in cl.Tree.checked:
                                                     #print(a.predicate)   
					             #print(a.predicate)
                                                     return a
		return False 

	def abstract_precondition1(self,p,obj):
		if p in preconditions.predicates:
			     for each in obj:
				if not each in preconditions.objs :
					return False
				return True
		else:
			return False		

	def abstract_precondition2(self,obj):
		result = []
		for item in cl.ObjectTree.allnodes:
			if item.predicate in obj:
				if len(item.parents):
					for temp in item.parents:
						result.append(temp.predicate)
						break
					
		if len(result):
			return result
		return False


	def concretion_precondition2(self,obj):
		result = []
		for item in cl.ObjectTree.allnodes:
			if item.predicate in obj:
				if len(item.children):
					for a in item.children:
                                               if not a.predicate in cl.ObjectTree.checked:
                                                     #print(a.predicate)   
					             #print(a.predicate)
							for temp in item.children:
								result.append(temp.predicate)
								break
		if len(result):
			return result
		return False
	
     
