"""
Blocks World* methods for Pyhop 1.1.
Author: Dana Nau <nau@cs.umd.edu>, November 15, 2012
This file should work correctly in both Python 2.7 and Python 3.2.
"""

"""
* Modified by Dustin Dannenhauer: Added a new operator for stacking with mortar
* Further Modified by Sravya Kondrakunta: Added a new operator get-from-store
"""

import pyhop

"""
Here are some helper functions that are used in the methods' preconditions.
"""

def is_done(b1,state,goal):
    if b1 == 'table': return True
    if b1 in goal.pos and goal.pos[b1] != state.pos[b1]:
        return False
    if state.pos[b1] == 'table': return True
    if state.pos[b1] == 'store': return True
    if state.pos[b1] in goal.pos.values() and (b1 not in goal.pos or goal.pos[b1] != state.pos[b1]):
        return False
    return is_done(state.pos[b1],state,goal)

def status(b1,state,goal):
    if is_done(b1,state,goal):
        return 'done'
#    elif not (state.clear[b1] or state.pos[b1] == "in-arm"):
#        return 'inaccessible'
   
    elif (b1 in goal.pos) and state.pos[b1] == 'store':
        return 'get-from-store'
    elif not (b1 in goal.pos) or goal.pos[b1] == 'table':
        return 'move-to-table'
    elif is_done(goal.pos[b1],state,goal) and state.clear[goal.pos[b1]]:
        btmblk = goal.pos[b1]
        if btmblk in goal.hasmortar.keys() and goal.hasmortar[btmblk]:
            ''' This check adds a status so we move blocks with mortar different then without mortar '''
            return 'move-to-block-with-mortar'
        else:
            return 'move-to-block'
    else:
        return 'waiting'

def all_blocks(state):
    return state.clear.keys()

def get_by_value(dictionary,val):
    block = None
    for key, value in dictionary.iteritems():
	 if value == val:
	 	block = key
    return block
     
def check_all_blocks_in_goal(table_block,goal_pos,blocks):
    if not table_block == None:
	#print(blocks)
	blocks.append(table_block)
	table_block = get_by_value(goal_pos,table_block)
	#print(table_block)
	return check_all_blocks_in_goal(table_block,goal_pos,blocks)
    else:
	return blocks

def blocks_according_to_goal(state,goal):
    blocks = []
    all_blocks = []
    table_block = get_by_value(goal.pos,"table")
    blocks = check_all_blocks_in_goal(table_block,goal.pos,blocks)
    for each in state.clear.keys():
	if not each in blocks:
		blocks.append(each) 
    return blocks


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
    blocks = blocks_according_to_goal(state,goal)
    #print(blocks)
    for b1 in blocks:
	s = status(b1,state,goal)
#	print("status for block" + str(b1) + "is " + str(s))
        if s == 'move-to-table':
            return [('move_one',b1,'table'),('move_blocks',goal)]
        elif s == 'move-to-block':
            return [('move_one',b1,goal.pos[b1]), ('move_blocks',goal)]
        elif s == 'move-to-block-with-mortar':
            return [('move_one_mortar',b1,goal.pos[b1]), ('move_blocks',goal)]
	elif s == 'get-from-store':
            return [('move_from_store',b1,goal.pos[b1]), ('move_blocks',goal)]
        else:
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
    if state.pos[b1] == "in-arm":
        return [('put', b1,dest)]
    else:
        return [('get', b1), ('put', b1,dest)]

### methods for "move_one_mortar"

def move1_mortar(state,b1,dest):
    """
    Generate subtasks to get b1 and put it at dest (with mortar)
    """
    if state.pos[b1] == "in-arm":
        return [('put_mortar', b1,dest)]
    else:
        return [('get', b1), ('put_mortar', b1,dest)]

### methods for "move_from_store"

def move_store(state,b1,dest):
    """
    Generate subtasks to get b1 and put it at dest.
    """
    if dest == "table":
        return [('get_from_store',b1)]
    else:
	if(state.pos[dest] == "store"):
		return [('get_from_store',dest), ('put', dest,'table') , ('get_from_store',b1)]

	return [('get_from_store',b1)]

### methods for "get"

def get_by_unstack(state,b1):
    """Generate a pickup subtask."""
    if state.clear[b1]: return [('unstack_task',b1)]
    return False

def get_by_pickup(state,b1):
    """Generate a pickup subtask."""
    if state.clear[b1]: return [('pickup_task',b1)]
    return False

### methods for "pickup_task"

def pickup_m(state,b1):
    """Generate a pickup subtask."""
    if state.clear[b1]: return [('pickup',b1)]
    return False

### methods for "unstack_task"

def unstack_m(state,b1):
    """Generate a pickup subtask."""
    if state.clear[b1]:
        btmblk = state.pos[b1]
        mortarblk = state.hasmortar[btmblk] 
        if mortarblk: 
            return [('unstack_mortared',b1,state.pos[b1],mortarblk)]
        else:
            return [('unstack',b1,state.pos[b1])]
    return False

### methods for "put"

def put_m(state,b1,b2):
    """
    Generate either a putdown or a stack subtask for b1.
    b2 is b1's destination: either the table or another block.
    """
    
    if state.holding == b1:
        if b2 == 'table':
            return [('putdown',b1)]
        else:
            return [('stack',b1,b2)]
    else:
        return False
            
### Methods for put_mortar
            
def put_m_mortar(state,b1,b2):
    """
    Generate either a putdown or a stack subtask for b1.
    b2 is b1's destination: either the table or another block.
    """
    
    available_mortar = [k for k,v in state.mortaravailable.items() if v]
    mortar_block = False
    if len(available_mortar) > 0:
        mortar_block = available_mortar[0]
    
    if state.holding == b1:
        if b2 == 'table':
            return [('putdown',b1)]
        elif mortar_block:
            # new stack with mortar
            #print("*-*-*-*-*-* stacking with mortar")
            return [('stack_mortared',b1,b2,mortar_block)]
        else:
            # no mortar left, fail
            #print("      *******\n       NO MORE MORTAR\n      *******")
            return False
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

def declare_methods(longApprehend = True):
    if longApprehend:
        pyhop.declare_methods("catch_arsonist", long_apprehend_m)
    else:
        pyhop.declare_methods("catch_arsonist", quick_apprehend_m)
    pyhop.declare_methods("put_out", put_out_m)
    pyhop.declare_methods('put',put_m)
    pyhop.declare_methods('put_mortar',put_m_mortar)
    pyhop.declare_methods('unstack_task',unstack_m)
    pyhop.declare_methods('pickup_task',pickup_m)
    pyhop.declare_methods('get',get_by_pickup,get_by_unstack)
    pyhop.declare_methods('move_one',move1)
    pyhop.declare_methods('move_one_mortar',move1_mortar)
    pyhop.declare_methods('move_from_store',move_store)
    pyhop.declare_methods('move_blocks',moveb_m)
#    pyhop.declare_methods('put_with_mortar',put_with_mortar_m)
