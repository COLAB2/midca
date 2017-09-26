"""
Blocks World domain definition for Pyhop 1.1.
Author: Dana Nau <nau@cs.umd.edu>, November 15, 2012
This file should work correctly in both Python 2.7 and Python 3.2.
"""



import sys
sys.path.append("../")
import pyhop
import random 

"""Each Pyhop planning operator is a Python function. The 1st argument is
the current state, and the others are the planning operator's usual arguments.
This is analogous to how methods are defined for Python classes (where
the first argument is always the name of the class instance). For example,
the function pickup(state,b) implements the planning operator for the task
('pickup', b).

The blocks-world operators use three state variables:
- pos[b] = block b's position, which may be 'table', 'in-arm', or another block.
- clear[b] = False if a block is on b or the in-arm is holding b, else True.
- holding = name of the block being held, or False if the in-arm is empty.
"""

def pickup(state,b):
    if state.pos[b] == 'table' and state.clear[b] == True and state.holding == False:
        state.pos[b] = 'in-arm'
        state.clear[b] = False
        state.holding = b
        
        # remove mortar if the block has it (just to be safe)
        if hasattr(state, 'mortared') and state.mortared[b] == True:
            state.mortared[b] == False
        
        return state
    else: return False

def unstack(state,b,c):
    if state.pos[b] == c and c != 'table' and state.clear[b] == True and state.holding == False:
        state.pos[b] = 'in-arm'
        state.clear[b] = False
        state.holding = b
        state.clear[c] = True

        # to undo mortar but doesn't increase quantity
        if hasattr(state, 'mortared') and state.mortared[b] == True:
            state.mortared[b] == False
        
        return state
    else: return False
    
def unstack_mortared(state,b,c,m):    
    ''' m is for mortar '''
    if state.pos[b] == c and c != 'table' and state.clear[b] == True and state.holding == False:
        state.pos[b] = 'in-arm'
        state.clear[b] = False
        state.holding = b
        state.clear[c] = True
        if state.hasmortar[b] == m:
            state.hasmortar[b] = False
        
        return state
    else: return False

def putdown(state,b):
    if state.pos[b] == 'in-arm':
        state.pos[b] = 'table'
        state.clear[b] = True
        state.holding = False
        return state
    else: return False

def stack(state,b,c):
    if state.pos[b] == 'in-arm' and state.clear[c] == True:
        state.pos[b] = c
        state.clear[b] = True
        state.holding = False
        state.clear[c] = False
        return state
    else: return False

def stack_mortared(state,b,c,m):
    if state.pos[b] == 'in-arm' and state.clear[c] == True and state.mortaravailable[m]:
        state.pos[b] = c
        state.clear[b] = True
        state.holding = False
        state.clear[c] = False
        state.hasmortar[b] = True
        state.mortaravailable[m] = False
        return state
    else: return False    
    
def putoutfire(state, b):
	if state.fire[b] == True:
		state.fire[b] == False
		return state
	else: 
		return False

def apprehend(state, perp):
	if state.free[perp] == True:
		state.free[perp] = False
		return state
	else:
		return False

def searchfor(state, perp):
	return state

def get_from_store(state,b):
	if state.pos[b] == 'store' and state.clear[b] == True and state.holding == False:
        	state.pos[b] = 'in-arm'
        	state.clear[b] = False
        	state.holding = b
        	return state
    	else: 
		return False

"""
Below, 'declare_operators(pickup, unstack, putdown, stack)' tells Pyhop
what the operators are. Note that the operator names are *not* quoted.
"""

def declare_ops():
	pyhop.declare_operators(pickup, unstack, unstack_mortared, putdown, stack, stack_mortared, putoutfire, apprehend, searchfor, get_from_store)
