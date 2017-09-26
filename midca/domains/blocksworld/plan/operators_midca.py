"""
Blocks World domain definition for Pyhop 1.1.
Author: Dana Nau <nau@cs.umd.edu>, November 15, 2012
This file should work correctly in both Python 2.7 and Python 3.2.
"""

from midca.modules._plan import pyhop

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
    print "pickop_op", b
    if state.is_true("on-table", [b]) and state.is_true("clear", [b]) and state.is_true("arm-empty"):
        state.add_fact("holding", [b])
        state.remove_fact("clear", [b])
        state.remove_fact("on-table", [b])
        state.remove_fact("arm-empty")
        return state
    else: return False

def unstack(state,b,c):
    if state.is_true("on", [b, c]) and c != 'table' and state.is_true("clear", [b]) and state.is_true("arm-empty"):
        state.add_fact("holding", [b])
        state.remove_fact("clear", [b])
        state.remove_fact("on", [b, c])
        state.remove_fact("arm-empty")
        state.add_fact("clear", [c])
        return state
    else: return False
    
def putdown(state,b):
    if state.is_true("holding", [b]):
        state.add_fact("on-table", [b])
        state.add_fact("arm-empty")
        state.add_fact("clear", [b])
        state.remove_fact("holding", [b])
        return state
    else: return False

def stack(state,b,c):
    if state.is_true("holding", [b]) and state.is_true("clear", [c]):
        state.remove_fact("holding", [b])
        state.add_fact("clear", [b])
        state.add_fact("on", [b, c])
        state.add_fact("arm-empty")
        state.remove_fact("clear", [c])
        return state
    else: return False

def putoutfire(state, b):
	if state.is_true("onfire", [b]):
		state.remove_fact("onfire", [b])
	else: 
		return False

def apprehend(state, perp):
	if state.is_true("free", [perp]):
		state.remove_fact("free", [perp])
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
	pyhop.declare_operators(pickup, unstack, putdown, stack, putoutfire, apprehend, searchfor)
