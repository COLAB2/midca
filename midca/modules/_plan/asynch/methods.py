from midca.modules._plan import pyhop

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

def all_blocks(state):
    #return state.all_objects()
    return ['green block', 'red block']
	
def achieve_goals_m(state, goals):
	print("achieve_goals_m")
	
	if goals:
		goal = goals[0]
		get_goal_pos(goal)
		object = goal["directObject"]
		#print(object + " is " + get_last_clear_status(state, object))
		if goal["objective"] == "show-loc":
			return [("point_at", goal["directObject"]), ("achieve_goals", goals[1:])]
		
		if goal["objective"] == "stacking":
			print("holding")
			return [("move_blocks", goal)]
		
		if goal["objective"] == "holding":
 			if get_last_clear_status(state, object) == 'clear':
			 	return [("pickup_task", goal["directObject"]), ("achieve_goals", goals[1:])]
		
		
		if goal["objective"] == "moving":
 			if get_last_clear_status(state, object) == 'clear':
			 	return [('move_one',goal["directObject"],'table'), ("achieve_goals", goals[1:])]
		
			 
#  			else:
#  				return [("unstack_t", "green block", goal["directObject"]), ("pickup", goal["directObject"]), ("achieve_goals", goals[1:])]
#  				
					
				
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
	#print("block: " + b1)
	if b1 == 'table': return True
	if b1 in goal_pos_dic:
		print(goal_pos_dic[b1])
	else:
		print("no!")
	if b1 in goal_pos_dic and str(goal_pos_dic[b1]) != str(get_last_position(state, b1)):
		print("return false")
		return False
	if get_last_position(state, b1)== 'table': return True
	if get_last_position(state, b1) in goal_pos_dic.values() and (b1 not in goal_pos_dic or goal_pos_dic[b1] != get_last_position(state, b1)):
		return False
	raw_input('Enter ...')
	return is_done(get_last_position(state, b1),state,goal)

def status(b1,state,goal):
	print("status" + b1)
	if b1 in goal_pos_dic:
		print(goal_pos_dic[b1])
	print("***********")
	if is_done(b1,state,goal):
		print("done")
		return 'done'
	elif not (get_last_clear_status(state, b1) or get_last_position(state, b1) == "hand"):
		rprint('inaccessible')
		return 'inaccessible'
	elif not (b1 in goal_pos_dic) or str(goal_pos_dic[b1]).strip() == 'table':
		print("move to table")
		return 'move-to-table'
	elif is_done(goal_pos_dic[b1],state,goal) and get_last_clear_status(state, goal_pos_dic[b1]):
		print("move to block")
		return 'move-to-block'
	else:
		print("waiting")
		return 'waiting'



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
    	print("___block: " + b1)
    	raw_input('Enter ...')
        s = status(b1,state,goal)
        if s == 'move-to-table':
        	print("___move one")
        	return [('move_one',b1,'table'),('move_blocks',goal)]
        elif s == 'move-to-block':
            return [('move_one',b1,goal_pos_dic[b1]), ('move_blocks',goal)]
        else:
        	print("continue")
        	continue
    #
    # if we get here, no blocks can be moved to their final locations
    b1 = pyhop.find_if(lambda x: status(x,state,goal) == 'waiting', all_blocks(state))
    if b1 != None:
        return [('move_one',b1,'table'), ('move_blocks',goal)]
    #
    # if we get here, there are no blocks that need moving
    return []

"""
declare_methods must be called once for each taskname. Below, 'declare_methods('get',get_m)' tells Pyhop that 'get' has one method, get_m. Notice that 'get' is a quoted string, and get_m is the actual function.
"""

### methods for "move_one"

def move1(state,b1,dest):
    """
    Generate subtasks to get b1 and put it at dest.
    """
    if get_last_position(state, b1) == "in-arm":
    	return [('put', b1,dest)]
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
    if get_last_clear_status(state, b1) == 'clear' and get_last_position(state, b1) == 'table':
     	return [('pickup_task',b1)]
    elif get_last_clear_status(state, b1) == 'clear' and get_last_position(state, b1) != 'table':
    	return [('unstack_task',b1)]
     	
    #return False
    

def put_m(state,b1,b2):
    """
    Generate either a putdown or a stack subtask for b1.
    b2 is b1's destination: either the table or another block.
    """
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
	pyhop.declare_methods("point_at", point_at_m)
	pyhop.declare_methods("achieve_goals", achieve_goals_m)
	pyhop.declare_methods("put_out", put_out_m)
	pyhop.declare_methods('put',put_m)
	pyhop.declare_methods('unstack_task',unstack_m)
	pyhop.declare_methods('pickup_task',pickup_m)
	pyhop.declare_methods('get',get_m)
	pyhop.declare_methods('move_one',move1)
	pyhop.declare_methods('move_blocks',moveb_m)
