from MIDCA.modules._plan import pyhop

def point_at_m(state, objectID):
	return [("block_until_seen", objectID), ("point_at", objectID)]