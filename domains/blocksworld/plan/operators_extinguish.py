"""
Blocks World domain definition for Pyhop 1.1.
Author: Dana Nau <nau@cs.umd.edu>, November 15, 2012
This file should work correctly in both Python 2.7 and Python 3.2.
"""
import sys
sys.path.append("../")
from MIDCA.modules._plan import pyhop

"""Each Pyhop planning operator is a Python function. The 1st argument is
the current state, and the others are the planning operator's usual arguments.
This is analogous to how methods are defined for Python classes (where
the first argument is always the name of the class instance). For example,
the function pickup(state,b) implements the planning operator for the task
('pickup', b).

The blocks-world operators use three state variables:
- pos[b] = block b's position, which may be 'table', 'hand', or another block.
- clear[b] = False if a block is on b or the hand is holding b, else True.
- holding = name of the block being held, or False if the hand is empty.
"""

def pickup(state,b):
    if state.pos[b] == 'table' and state.clear[b] == True and state.holding == False:
        state.pos[b] = 'hand'
        state.clear[b] = False
        state.holding = b
        return state
    else: return False

def unstack(state,b,c):
    if state.pos[b] == c and c != 'table' and state.clear[b] == True and state.holding == False:
        state.pos[b] = 'hand'
        state.clear[b] = False
        state.holding = b
        state.clear[c] = True
        return state
    else: return False
    
def putdown(state,b):
    if state.pos[b] == 'hand':
        state.pos[b] = 'table'
        state.clear[b] = True
        state.holding = False
        return state
    else: return False

def stack(state,b,c):
    if state.pos[b] == 'hand' and state.clear[c] == True:
        state.pos[b] = c
        state.clear[b] = True
        state.holding = False
        state.clear[c] = False
        return state
    else: return False

def putoutfire(state, b, ext):
	if state.fire[b] == True and state.holdingfireext == ext:
		state.fire[b] == False
		return state
	else: 
		return False

def pickup_extinguisher(state, extinguisher):
	if extinguisher in state.fire_ext_avail and not state.holdingfireext:
		state.holdingfireext = extinguisher
		state.fire_ext_avail.remove(extinguisher)
		return state
	else:
		return False

def putdown_extinguisher(state, extinguisher):
	if state.holdingfireext == extinguisher:
		state.holdingfireext = None
		state.fire_ext_avail.add(extinguisher)
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

"""
Below, 'declare_operators(pickup, unstack, putdown, stack)' tells Pyhop
what the operators are. Note that the operator names are *not* quoted.
"""
def declare_ops():
	pyhop.declare_operators(pickup, unstack, putdown, stack, putoutfire, apprehend, searchfor, pickup_extinguisher, putdown_extinguisher)
