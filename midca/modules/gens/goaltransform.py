from midca import  goals

def identity(mem,goal,canexec):
	'''
	get the world from the mem variable and return the parameter as it is identity ,The canexec is 0 for choose checking it else it should 		be 1 
	'''
	world = mem.myMidca.midca.world
	cond = preconditions_all(mem)
	pre = cond.identity_conditions(world)
	return goal

def generalization(mem,goal,canexec):
	'''
	get the world from memory , create a dictionary of its preconditions and check the second precondition and later both the first and 		the third preconditions . The second precondition returns the parent node , while the first checks whether the predicate and the 		objects are in the world or not . The third pre condition checks the resources. The canexec is 0 for choose checking it else it should 		be 1 
	'''
	world = mem.myMidca.midca.world
	cond = preconditions_all(mem)
	preconditions = {'pre': cond.genral_precondition1, 'pre1': cond.genral_precondition2, 'pre2': cond.genral_precondition3}
	p1 = preconditions['pre1'](world,goal.get_pred())
	if p1 == False:
		return False
	else:
	 	result_goal = goals.Goal(*goal.get_args(), predicate = p1.predicate)
	if preconditions['pre'](world,goal.get_pred(),goal.get_args())  and preconditions['pre2'](mem) :
	     	if canexec:
                	world.cltree['checked'].append(p1.predicate)
	      	return result_goal
	else:
		return False


def specailization(mem,goal,canexec):
	'''
	get the world from memory , create a dictionary of its preconditions and check the second precondition and later both the first and 		the third preconditions . The second precondition returns the children node , while the first checks whether the predicate and the 		objects are in the world or not . The third pre condition checks the resources. The canexec is 0 for choose checking it else it should 		be 1 
	'''
	world = mem.myMidca.midca.world
	cond = preconditions_all(mem)
	preconditions = {'pre': cond.genral_precondition1, 'pre1': cond.specail_precondition2, 'pre2': cond.genral_precondition3}
	p1 = preconditions['pre1'](world,goal.get_pred())
	if p1 == False:
		return False
	
	else:
		result_goal = goals.Goal(*goal.get_args(), predicate = p1.predicate)
	
	if preconditions['pre'](world,goal.get_pred(),goal.get_args()) and preconditions['pre2'](mem):
		if canexec: 
                	world.cltree['checked'].append(p1.predicate)
		return result_goal
	else:
		return False



def choose(mem,goal):
	'''
	in choose there will be delta and deltahat , delta contains all the goal tranformation functions that should be checked on a goal and 		the deltahat contains only the applicable goal transformations . First we check delta and we remove the functions that return false 		and copy them into deltahat, finally execute delta hat and return the result . The canexec variable is 0 if it is executing delta and 		it is 1 if we are executing deltahat
	'''
	print("*************** GOAL TRANSFORMATION CHOOSE STARTS *********************\n")
	world = mem.myMidca.midca.world
	delta = {'identity' : identity ,'generalization' : generalization ,'specailization' : specailization}
	item = ['identity' , 'generalization' , 'specailization']
	deltahat = {}
	temp_parameter = goal
	resitem = list()
	res = goal
	print("Delta contains functions: " + str(item))
	print("Checking each function in Delta: ")
	for i in item:
		if delta[i](mem,goal,canexec = 0) is False:
			continue
		else:
			temp_parameter = delta[i](mem,goal,canexec = 0)
			print(str(i)  + ": " + str(temp_parameter))
			deltahat[i] = i
			resitem.append(i)
	resitem.reverse()
	print("\nDeltahat contains functions: " + str(resitem) )
	resitem.reverse()
	print("\nExecuting each function in Deltahat: ")
	for i in resitem:
		res = delta[i](mem,res,canexec = 1)
		print(str(i)  + ": " + str(res))	

	if resources.num_available_mortar <= 0:
		resources.num_available_mortar = 0
	else:
		if "stable-on" == res["predicate"]:
			resources.num_available_mortar -=1
	
	print("\nThe Final Transformed Goal is : " + str(res))
	print("\n*************** GOAL TRANSFORMATION CHOOSE ENDS *********************\n")
	return res

def abstraction(mem,parameter,canexec):
	world = mem.myMidca.midca.world
	cond = preconditions_all(mem)
	preconditions = {'pre': cond.abstract_precondition1, 'pre1': cond.abstract_precondition2}
	index = parameter.find("(")
	p = parameter[ : index]
	index1 = parameter.find(",")
	index2 = parameter.find(")")
        obj = parameter[index+1:index2].split(",")
	obj1 = preconditions['pre1'](world,obj)
	if obj1 == False:
		return False
	else:
		
	 	res = p + "(" + ",".join(obj1) + ")"
	if preconditions['pre'](world,p,obj):
		    if canexec:
			for a in obj1:
                		world.obtree['checked'].append(a)
		    return res
	else:
		return False

def concretion(mem,parameter,canexec):
	world = mem.myMidca.midca.world
	cond = preconditions_all(mem)
	preconditions = {'pre': cond.abstract_precondition1, 'pre1': cond.concretion_precondition2}
	index = parameter.find("(")
	p = parameter[ : index]
	index1 = parameter.find(",")
	index2 = parameter.find(")")
	obj = parameter[index+1:index2].split(",")	
	obj1 = preconditions['pre1'](world,obj)
	if obj1 == False:
		return False
	else:
		
	 	res = p + "(" + ",".join(obj1) + ")"
	if preconditions['pre'](world,p,obj):
		    if canexec:
			for a in obj1:
                		world.obtree['checked'].checked.append(a)
		    return res
	else:
		return False

class resources:
	num_available_mortar=0
	def __init__(self,mem):
		resources.num_available_mortar = 0
		for atom in map(str, mem.get("__world states")[-1].get_atoms()):
                	if 'available(' in atom:
                    		resources.num_available_mortar +=1

class preconditions_all:
	'''
	This class is for preconditions of all functions , the objs and predicates are the datamembers that are initialized from the objects 		and predicates of the world 
	'''
	
	objs = []
	predicates = []

	def __init__(self,mem):
		world = mem.myMidca.midca.world
		for a in world.objects.values():
			self.objs.append(a.name)
		for a in world.predicates.values():
			self.predicates.append(a.name)

	def identity_conditions(self,world):
		return True


	def genral_precondition1(self,world,p,obj):
		'''
		generalization precondition 1 which checks whether the given objects in the goal are in the objects of the world or not
		'''
		for item in world.cltree['allnodes']:
			if item.predicate == p:
			     for each in obj:
				if not each in preconditions_all.objs :
					return False
			     return True
		else:
			return False

	def genral_precondition2(self,world,p):
		'''
		generalization precondition 2 which returns the parent  of the given predicate in the goal , using the class 			heirarchy tree that is in the world
		'''
		for item in world.cltree['allnodes']:
			if item.predicate == p:
				if len(item.parents):
					return item.parents[0]
		return False 

	def genral_precondition3(self,mem):
	    	'''
	    	generalization precondition 3 which checks the resources
	    	'''
	    	if resources.num_available_mortar <= 0:
			return True
	    	return False

	def specail_precondition2(self,world,p):
		'''
		specailization precondition 2 which returns the children of the given predicate in the goal , using the class 			heirarchy tree that is in the world . If there are two to three children , if there exists a children as the result then it 			will return the next children that is not in the result of deltahat
		'''
		for item in world.cltree['allnodes']:
			if item.predicate == p:
				if len(item.children):
					for a in item.children:
                                               if not a.predicate in world.cltree['checked']:
                                                     return a
		return False 

	def abstract_precondition1(self,world,p,obj):
		if p in preconditions.predicates:
			     for each in obj:
				if not each in preconditions_all.objs :
					return False
				return True
		else:
			return False		

	def abstract_precondition2(self,world,obj):
		result = []
		for item in world.obtree['allnodes']:
			if item.predicate in obj:
				if len(item.parents):
					for temp in item.parents:
						result.append(temp.predicate)
						break
					
		if len(result):
			return result
		return False


	def concretion_precondition2(self,world,obj):
		result = []
		for item in world.obtree['allnodes']:
			if item.predicate in obj:
				if len(item.children):
					for a in item.children:
                                               if not a.predicate in world.obtree['checked']:
							for temp in item.children:
								result.append(temp.predicate)
								break
		if len(result):
			return result
		return False
	
     





