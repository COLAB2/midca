from MIDCA.modules._plan import pyhop


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

def declare_ops():
	pyhop.declare_operators(block_until_seen, point_to, block_until_complete)
