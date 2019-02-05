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
        if predicate == "reputed":
            return [ ('see_statistics',args[1],args[2],args[3]) , ('make_policy',args[0],args[1],args[2],args[3]), ('inform_policy',args[0], args[3]), ('implement_policy',args[0],args[1],args[2],args[3]), ('achieve_goals', goals[1:])]
        else:
            return []
    return []



def declare_methods(longApprehend = True):
    pyhop.declare_methods("achieve_goals", achieve_goals)
