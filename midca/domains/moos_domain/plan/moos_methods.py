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

	if predicate == "hazard_checked":
		vehicle = state.enabled.pop()
		return[('check',args[0],args[1],vehicle) , ('achieve_goals', goals)]
    return []


def check_hazard(state,mine,location,vehicle):
    '''
    Go through each goal and create an order for each goal
    '''

    if location == "qroute":
	return[('remove',mine,location,vehicle)]
    else:
	return[('ignore',mine,location,vehicle)]

    return []

def declare_methods(longApprehend = True):
    pyhop.declare_methods("achieve_goals", achieve_goals)
    pyhop.declare_methods("check", check_hazard)
