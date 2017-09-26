from midca.modules._plan import modified_pyhop
import time


def block_until_seen(state, objectID):
	'''
	blocks until the given object is observed. Parameters defining how recently the 
	observation must have occurred and/or an upper limit on the amount of time to wait
	before the action is determined to have failed should be defined elsewhere.
	'''
	return state

def point_to(state, objectID):
	return state

def block_until_complete(state):
	'''
	blocks until the action preceding this one is complete. If this action begins a plan,
	it will have no effect. 
	'''
	return state


def grab(state, objectID):
	return state

def release(state, objectID):
	return state


def putdown(state,b):
    if get_last_position(state, b) == 'in-arm':
        set_position(state, b, 'table')
        set_clear_status(state, b, 'clear')
        #state.holding = False
        return state
    else: return False

def raising(state, objectID):
	return state

def raising_arm(state):
	return state

def reach_to_pickup(state,b):
    #time.sleep(2)
    #if get_last_position(state, b) == 'table' and get_last_clear_status(state, b) == 'clear':
    if get_last_clear_status(state, b) == 'clear':
        #state.position[b] = 'hand'
        set_position(state, b, 'in-arm')
        #state.isclear[b] = False
        set_clear_status(state, b, 'not clear')
        #state.holding = b
        return state
    else: return False

def reach_to_unstack(state,b,c):
    #time.sleep(2)
    if get_last_position(state, b) == c and get_last_clear_status(state, b) == 'clear':
        set_position(state, b, 'in-arm')
        set_clear_status(state, b, 'not clear')
        #state.holding = b
        set_clear_status(state, c, 'clear')
        return state
    else: return False

def stack(state,b,c):
    #if get_last_position(state, b) == 'in-arm' and get_last_clear_status(state, b) == 'clear':
    #time.sleep(2)
    #raw_input('Enter ...')
    if get_last_clear_status(state, c) == 'clear':
    	print("yes it is clear")
        set_position(state, b, c)
        set_clear_status(state, b, 'clear')
        #state.holding = False
        set_clear_status(state, c, 'not clear')
        return state
    else:
    	print("return false") 
    	return False


def get_last_position(state, objectOrID):
	
	positions = state.all_pos(objectOrID)
	if not positions:
		return None
	else:
		for state_pos in reversed(positions):
			if state_pos.position:
				return (state_pos.position)
	return None

def set_position(state, objectOrID, newPos):
	
	positions = state.all_pos(objectOrID)
	if not positions:
		return None
	else:
		for state_pos in reversed(positions):
			if state_pos.position:
				state_pos.position = newPos
				return state
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

def set_clear_status(state, objectOrID, newClearStatus):
	
	positions = state.all_pos(objectOrID)
	if not positions:
		return None
	else:
		for state_pos in reversed(positions):
			if state_pos.isclear:
				state_pos.isclear = newClearStatus
				return state
	return None



def declare_ops():
	modified_pyhop.declare_operators(block_until_seen, point_to, block_until_complete,
						 grab, raising, putdown, reach_to_pickup, reach_to_unstack, stack, release, raising_arm)


