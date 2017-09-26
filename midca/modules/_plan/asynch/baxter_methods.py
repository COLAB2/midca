from midca.modules._plan import pyhop

def point_at_m(state, objectID):
	return [("block_until_seen", objectID), ("point_to", objectID)]
def pickup(state, objectID):
	#if get_last_position(state, objectID) == "table":
	return [("reach", objectID), ("grab", objectID), ("raising", objectID)]
	#return [("block_until_seen", objectID), ("reach", objectID)]
def unstack_task(state, b1, b2):
	#if get_last_position(state, b1)== b2:
	return [("unstack", b1, b2), ("grab", b1), ("putdown", b1)]

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
	
def achieve_goals_m(state, goals):
	if goals:
		goal = goals[0]
		object = goal["directObject"]
		print(object + " is " + get_last_clear_status(state, object))
		if goal["objective"] == "show-loc":
			return [("point_at", goal["directObject"]), ("achieve_goals", goals[1:])]
		if goal["objective"] == "holding":
 			if get_last_clear_status(state, object) == 'clear':
			 	return [("pickup", goal["directObject"]), ("achieve_goals", goals[1:])]
 			else:
 				return [("unstack_t", "green block", goal["directObject"]), ("pickup", goal["directObject"]), ("achieve_goals", goals[1:])]
 				
					
				
		else:
			return False #fail if goal is not of known type
	return [] #return empty plan if no goals.

def declare_methods():
	pyhop.declare_methods("point_at", point_at_m)
	pyhop.declare_methods("pickup", pickup)
	pyhop.declare_methods("unstack_t", unstack_task)
	pyhop.declare_methods("achieve_goals", achieve_goals_m)
