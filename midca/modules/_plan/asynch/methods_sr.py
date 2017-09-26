from midca.modules._plan import modified_pyhop
import time 

def point_at_m(state, objectID):
	return [("block_until_seen", objectID), ("point_to", objectID)]

def pickup_m(state, objectID):
	#if get_last_position(state, objectID) == "table":
	#if get_last_clear_status(state, object) == 'clear':
	return [("reach_to_pickup", objectID), ("grab", objectID), ("raising", objectID)]
	#return False
	#return [("block_until_seen", objectID), ("reach", objectID)]
def unstack_m(state, b1):
	#if get_last_position(state, b1)== b2:
	#if get_last_clear_status(state, object) == 'clear':
	return [("reach_to_unstack", b1, get_last_position(state, b1)), ("grab", b1), ("raising", b1)]
	#return False

def get_last_position(state, objectOrID):

	positions = state.all_pos(objectOrID)
	if not positions:
		return None
	else:
		
		for state_pos in reversed(positions):
			if state_pos.position:
				return (state_pos.position)
	return None

def get_last_clear_status(state, objectOrID):
	
	positions = state.all_pos(objectOrID)
	if not positions:
		return None
	else:
		for state_pos in reversed(positions):
			if state_pos.isclear:
				return (state_pos.isclear)
	return None

def get_top_status(state, objectOrID):
	allobject = all_blocks(state)
	for each in allobject:
		if(get_last_position(state,each) == objectOrID):
			return each
	return False

def get_status_in_arm(state):
	allobject = all_blocks(state)
	block = None
	for each in allobject:
		if(get_last_position(state,each) == "in-arm"):
			block = each
			break
	if block is None:
		return False

	return block

def get_max_height_position_block(state , objectOrID):
	'''
	print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
	print("in max height position")
	print("the object is")
	print(objectOrID)
	print("get top status")
	print(get_top_status(state, objectOrID))
	print("get last clear status")
	print(get_last_clear_status(state,objectOrID ))
	'''
	print("the object is ")
	print(objectOrID)

	print("the top status is ")
	print(get_top_status(state, objectOrID))


	if (get_last_clear_status(state, objectOrID) == 'clear'):
		print("Result is ")
		print("")		
		print(objectOrID)
		return objectOrID
	else:
		return(get_max_height_position_block(state , get_top_status(state, objectOrID)))

	
	

def giveindirectobject(state,directobject):
	for each in all_blocks(state):
		if each in goal_pos_dic and not (each == directobject):
			return each
			break
	
	return False

def all_blocks(state):
    return state.all_objects()
    #return ['green block', 'red block' , 'blue block']
	
def achieve_goals_m(state, goals):
	print("achieve_goals_m")

	if goals:
		g = goals
		goal = goals[0]
		get_goal_pos(goal)
		object = goal["directObject"]
		print("goal objective is " + goal["objective"] )
		#print(object + " is " + get_last_clear_status(state, object))
		if goal["objective"] == "show-loc":
			return [("point_at", goal["directObject"]), ("achieve_goals", goals[1:])]
		
		if goal["objective"] == "stacking":
			print("holding")
			return [("move_blocks", goal), ("achieve_goals", goals[1:])]
		
		if goal["objective"] == "holding":
			'''
			print("STATUS")
			print(" last clear status of green block")
			print(get_last_clear_status(state, 'green block'))
			print(" last clear status of blue block")
			print(get_last_clear_status(state, 'blue block'))
			print(" last clear status of red block")
			print(get_last_clear_status(state, 'red block'))

			print("")
			print("")
			raw_input("status")
			
			print("")
			print("")
			print("------------------------------------------------------")
			print("Position")
			print(" last position of green block")
			print(get_last_position(state, 'green block'))
			print(" last position of blue block")
			print(get_last_position(state, 'blue block'))
			print(" last position of red block")
			print(get_last_position(state, 'red block'))

			
			print("")
			print("")
			raw_input("status")
			
			print("***********************************************************")
			print("in holding")
			print("the object is" )
			print(object)
			print("last clear status of object")
			print(get_last_clear_status(state, object))
			print("get last position of direct object")
			print(get_last_position(state, goal["directObject"]))
			print("get maximum height position")
			
			print("*******************************************************")
			print(get_max_height_position_block(state , object))
			print("")
			raw_input("enter")
			'''
			
			if get_last_position(state, goal["directObject"]) == 'in-arm':
						return [("achieve_goals", goals[1:])]
			
			if not get_status_in_arm(state):

 				if get_last_clear_status(state, object) == 'clear':
			 		return [("pickup_task", goal["directObject"]), ("achieve_goals", goals[1:])]
				else:

					return [("move_one", get_max_height_position_block(state , object) , 'table'),("achieve_goals", [goal]), ("achieve_goals", goals[1:])]
													
			else:


				return [("move_one", get_status_in_arm(state) , 'table'),("achieve_goals", [goal]), ("achieve_goals", goals[1:])]
				
				
		
		if goal["objective"] == "moving":
			
			if get_last_position(state, goal["directObject"]) == 'table':
						return [("achieve_goals", goals[1:])]

			elif not get_status_in_arm(state):

 				if get_last_clear_status(state, object) == 'clear':
			 		return [('move_one',goal["directObject"],'table'), ("achieve_goals", goals[1:])]
				else:

						return [("move_one", get_max_height_position_block(state , object) , 'table'),("achieve_goals", [goal]), ("achieve_goals", goals[1:])]
													
			else:
				if get_last_position(state, goal["directObject"]) == 'table':
						return [("achieve_goals", goals[1:])]

				return [("move_one", get_status_in_arm(state) , 'table'),("achieve_goals", [goal]), ("achieve_goals", goals[1:])]

			'''
 			if get_last_clear_status(state, object) == 'clear':
			 	return [('move_one',goal["directObject"],'table'), ("achieve_goals", goals[1:])]
 			if get_last_clear_status(state, object) == 'not clear':
				if get_last_position(state, object) == "in-arm":	
			 		return [('move_one',goal["directObject"],'table'), ("achieve_goals", goals[1:])]
				else:
					return [('move_one',get_top_status(state, object)),('move_one',goal["directObject"],'table'), ("achieve_goals", goals[1:])]
		
			 
#  			else:
#  				return [("unstack_t", "green block", goal["directObject"]), ("pickup", goal["directObject"]), ("achieve_goals", goals[1:])]
#  				
			'''		
				
		else:
			print("null")
			return False #fail if goal is not of known type
	return [] #return empty plan if no goals.
"""
Here are some helper functions that are used in the methods' preconditions.
"""
goal_pos_dic = {}

def get_goal_pos(goal):
	poses = goal["pos"]
	goal_pos_dic.update({poses.split(":")[0]: poses.split(":")[1]})
	 
	
def is_done(b1,state,goal):
	print("The given block is : " )
	print(b1)
	print("----------------------")
	if b1 == 'table': return True
	
	if b1 in goal_pos_dic:
		print(goal_pos_dic[b1])
	else:
		print("no!")
	
	if b1 in goal_pos_dic and str(goal_pos_dic[b1]) != str(get_last_position(state, b1)):
		#print("return false")
		return False
	if get_last_position(state, b1)== 'table': return True
	if get_last_position(state, b1) in goal_pos_dic.values() and (b1 not in goal_pos_dic or goal_pos_dic[b1] != get_last_position(state, b1)):
		return False
	#raw_input('Enter ...')
	return is_done(get_last_position(state, b1),state,goal)

def status(b1,state,goal):
	#print("***********")
	#print(get_last_clear_status(state, b1))
	direct_object = goal["directObject"]
	indirect_object = giveindirectobject(state,direct_object)
	status = get_last_clear_status(state, b1)
	last_position = get_last_position(state, b1)
	indirect_last_position = get_last_position(state, indirect_object)
	top_position = get_top_status(state, b1)
	direct_object_status = get_last_clear_status(state, direct_object)
	indirect_object_status = get_last_clear_status(state, indirect_object)
	status_in_arm = get_status_in_arm(state)	
	'''
	print("direct object is ")
	print(direct_object)	
	print("indirect object is ")
	print(indirect_object)
	print("status of the block")
	print(status)
	print("status in arm")
	print(status_in_arm)
	print("the block is ")
	print(b1)
	print("last position of block")
	print(last_position)
	print("top position of block")
	print(top_position)
	print("direct object status is ")
	print(direct_object_status)
	print("indirect object status is ")
	print(indirect_object_status)
	print("")
	print("")
	print("")
	print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
	'''
	#raw_input("enter")
	if indirect_last_position == direct_object:
		return 'done'	
	
	if status_in_arm:
		if b1 == status_in_arm:
			if b1 == direct_object :
#				print("In the condition of direct object in arm")
				return 'move-to-table'
			if b1 == indirect_object :
				print("In the condition of indirect object in arm")
				if direct_object_status == 'clear' :
#					print("direct object status is clear")
					return 'move-to-block'
				else:
#					print("direct object status is not clear")
					return 'move-to-table'

			if  ((not(b1 == direct_object)) and (not(b1== indirect_object)))  :
#				print("The  object in arm not in goal")
				return 'move-to-table'
	elif status == 'clear':
#		print("In the condition of status is clear")
		if  ((not(b1 == direct_object)) and (not(b1== indirect_object)))  :
#			print("In the condition of block not in goal")
			if last_position == direct_object or last_position == indirect_object:
#				print("In the condition of last position in direct object or indirect object")
				return 'move-to-table'
		if b1 == indirect_object:
#			print("In the condition of indirect object")
			if direct_object_status == 'clear':
#				print("In the condition of direct object status is clear")
				return 'move-to-block'
			elif last_position == direct_object:
#				print("In the condition of last_position == direct object")
				return 'done'
			elif  last_position == 'table':
				return 'waiting'
			else:
#				print("In the condition of else of last_position == direct object")
				return 'move-to-table'

		if b1 == direct_object:
			print("In the condition of direct object")
			
			if last_position == indirect_object :
#					print("In the condition of last_position == direct object")
				 	return 'move-to-table'	
			
			if (not last_position == 'table') and  (not (last_position == indirect_object))  and (not(get_last_clear_status(state, indirect_object) == 'clear')) :
				return 'move-to-table'

			else:
				return 'waiting'
			'''
			if top_position:
				print("top position is not none")
				if not (top_position == indirect_object):
					print("In the condition of else of last_position == direct object")
					return 'move-to-table'
			'''
					

	else :
		return 'waiting'

				

	
	'''
	if is_done(b1,state,goal):
		#print("done")
		return 'done'

	
		
		
	
	elif not (get_last_clear_status(state, b1) or get_last_position(state, b1) == "hand"):
		#print('inaccessible')
		return 'inaccessible'


	elif not (get_last_clear_status(state, b1) or get_last_position(state, b1) == "hand"):
		#print('inaccessible')
		return 'inaccessible'


	elif not (b1 in goal_pos_dic) or str(goal_pos_dic[b1]).strip() == 'table':
		#print("move to table")
		return 'move-to-table'

	elif is_done(goal_pos_dic[b1],state,goal) and get_last_clear_status(state, goal_pos_dic[b1]):
		#print("move to block")
		return 'move-to-block'
	

	else:
		#print("waiting")
		return 'waiting'

	'''

"""
In each Pyhop planning method, the first argument is the current state (this is analogous to Python methods, in which the first argument is the class instance). The rest of the arguments must match the arguments of the task that the method is for. For example, ('pickup', b1) has a method get_m(state,b1), as shown below.
"""

### methods for "move_blocks"

def moveb_m(state,goal):
    """
    This method implements the following block-stacking algorithm:
    If there's a block that can be moved to its final position, then
    do so and call move_blocks recursively. Otherwise, if there's a
    block that needs to be moved and can be moved to the table, then 
    do so and call move_blocks recursively. Otherwise, no blocks need
    to be moved.
    """
    
    for b1 in all_blocks(state):
#    	print("___block: " + b1)
    	#raw_input('Enter ...')
        s = status(b1,state,goal)
#	print("")
#	print("")
#	print("result")
#	print(s)
#	raw_input("enter")
        if s == 'move-to-table':
        	print("___move one")
        	return [('move_one',b1,'table'),('move_blocks',goal)]

				
        elif s == 'move-to-block':
#	    if not get_status_in_arm(state):
		return [('move_one',b1,goal_pos_dic[b1]), ('move_blocks',goal)]

        else:
		if s == 'done':
			return []
#			print("continue")
        		continue

    return []
    '''
    #
    # if we get here, no blocks can be moved to their final locations
    b1 = pyhop.find_if(lambda x: status(x,state,goal) == 'waiting', all_blocks(state))
    if b1 != None:
        return [('move_one',b1,'table'), ('move_blocks',goal)]
    #
    # if we get here, there are no blocks that need moving
    return []
    '''

"""
declare_methods must be called once for each taskname. Below, 'declare_methods('get',get_m)' tells Pyhop that 'get' has one method, get_m. Notice that 'get' is a quoted string, and get_m is the actual function.
"""

### methods for "move_one"

def move1(state,b1,dest):
    """
    Generate subtasks to get b1 and put it at dest.
    """
#    print("in move 1")
#    print(get_last_position(state, b1))
#    print(get_last_clear_status(state, b1))
    if get_last_position(state, b1) == "in-arm":
    	return [('put', b1,dest)]
    elif get_last_clear_status(state, b1) == 'not clear' :
	return [('unstack_task',get_top_status(state, b1)),('putdown',get_top_status(state, b1)),('release', get_top_status(state, b1)) ,('get', b1), ('put', b1,dest)]
    elif get_last_clear_status(state, dest) == 'not clear' :
	return [('unstack_task',get_top_status(state, dest)),('putdown',get_top_status(state, dest)),('release', get_top_status(state, dest)) ,('get', b1), ('put', b1,dest)]
	
    else:
    	return [('get', b1), ('put', b1,dest)]


### methods for "get"

def get_by_unstack(state,b1):
    """Generate a pickup subtask."""
    #if get_last_clear_status(state, b1) == 'clear':
    return [('unstack_task',b1)]
    #return False

def get_m(state,b1):
    """Generate a pickup subtask."""
    '''
    print("listing positions")
    print("green block")
    print(get_last_position(state, 'green block'))
    print("red block")
    print(get_last_position(state, 'red block'))
    print("blue block")
    print(get_last_position(state, 'blue block'))

    print("listing status")
    print("green block")
    print(get_last_clear_status(state, 'green block'))
    print("red block")
    print(get_last_clear_status(state, 'red block'))
    print("blue block")
    print(get_last_clear_status(state, 'blue block'))
    '''
    if get_last_clear_status(state, b1) == 'clear' and get_last_position(state, b1) == 'table':
     	return [('pickup_task',b1)]
    elif get_last_clear_status(state, b1) == 'clear' and get_last_position(state, b1) != 'table':
    	return [('unstack_task',b1)]
    elif get_last_clear_status(state, b1) == 'not clear' :
	return [('pickup_task',b1)]
  	
    return False
    

def put_m(state,b1,b2):
    """
    Generate either a putdown or a stack subtask for b1.
    b2 is b1's destination: either the table or another block.
    """

    '''
    print("listing positions")
    print("green block")
    print(get_last_position(state, 'green block'))
    print("red block")
    print(get_last_position(state, 'red block'))
    print("blue block")
    print(get_last_position(state, 'blue block'))

    print("listing status")
    print("green block")
    print(get_last_clear_status(state, 'green block'))
    print("red block")
    print(get_last_clear_status(state, 'red block'))
    print("blue block")
    print(get_last_clear_status(state, 'blue block'))
    '''

    if get_last_position(state, b1) == 'in-arm':
        if b2 == 'table':
            return [('putdown',b1), ('release', b1), ('raising', b1)]
        else:
            return [('stack',b1,b2), ('release', b1), ('raising', b1)]
    else:
        return False

def put_out_m(state, b1):
	if state.fire[b1]:
		return [("putoutfire", b1)]
	else:
		return []

def quick_apprehend_m(state, perp):
	if state.free[perp]:
		return [("apprehend", perp)]
	else:
		return []

def long_apprehend_m(state, perp):
	if state.free[perp]:
		return [("searchfor", perp), ("searchfor", perp), ("searchfor", perp), ("searchfor", perp), ("apprehend", perp)]
	else:
		return []
	
def declare_methods():
	modified_pyhop.declare_methods("point_at", point_at_m)
	modified_pyhop.declare_methods("achieve_goals", achieve_goals_m)
	modified_pyhop.declare_methods("put_out", put_out_m)
	modified_pyhop.declare_methods('put',put_m)
	modified_pyhop.declare_methods('unstack_task',unstack_m)
	modified_pyhop.declare_methods('pickup_task',pickup_m)
	modified_pyhop.declare_methods('get',get_m)
	modified_pyhop.declare_methods('move_one',move1)
	modified_pyhop.declare_methods('move_blocks',moveb_m)

