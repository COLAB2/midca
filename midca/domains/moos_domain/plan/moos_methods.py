from midca.modules._plan import pyhop


def achieve_goals(state,goals):
    '''
    Go through each goal and create an order for each goal
    '''
    if goals != []:
	goal = goals[0]
	#print(goal)
	predicate= goal['predicate']
	args = goal.args
	goals.remove(goal)
	if predicate == "at_location" and args[0] in state.enabled:
		return[('survey',args[0],args[1]) , ('achieve_goals', goals)]
    return []

def declare_methods(longApprehend = True):
    pyhop.declare_methods("achieve_goals", achieve_goals)

