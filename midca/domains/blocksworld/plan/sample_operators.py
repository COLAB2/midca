'''
Sample pyhop operators file. This is used to generate plans in the chicken domain.
'''

from midca.modules._plan import pyhop

def crossright(state, chicken):
    if state.is_true("onleft", [chicken]):
        state.remove_fact("onleft", [chicken])
        state.add_fact("onright", [chicken])
        return state
    else:
        return False

def crossleft(state, chicken):
    if state.is_true("onright", [chicken]):
        state.remove_fact("onright", [chicken])
        state.add_fact("onleft", [chicken])
        return state
    else:
        return False

def declare_ops():
	pyhop.declare_operators(crossright, crossleft)
