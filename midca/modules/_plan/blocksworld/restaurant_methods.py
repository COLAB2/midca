import pyhop

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
	if predicate == "order_serve":
		return[('create_order',args[0],args[1]) , ('achieve_goals', goals)]
    return []

def c_order(state,person,dish):
    return[('take_order', person, dish), ('prepare_order', person, dish), 
			('serve_order', person, dish)]

def declare_methods(longApprehend = True):
    pyhop.declare_methods("achieve_goals", achieve_goals)
    pyhop.declare_methods('create_order',c_order)
