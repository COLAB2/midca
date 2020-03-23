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
        if predicate == "cleared_mines" and args[0] in state.enabled:
            if args[1] == "ga1":
                if state.location == "transit1":
                    return[('slow_survey',args[0],args[1]), ('remove_mines',args[0],args[1]), ('achieve_goals', goals)]
                else:
                    return[('fast_survey',args[0],"transit1") , ('slow_survey',args[0],args[1]), ('remove_mines',args[0],args[1]), ('achieve_goals', goals)]
            elif args[1] == "ga2":
                if state.location == "qroute_transit":
                    return[('slow_survey',args[0],args[1]), ('remove_mines',args[0],args[1]), ('achieve_goals', goals)]
                else:
                    return[('fast_survey',args[0],"qroute_transit") , ('slow_survey',args[0],args[1]), ('remove_mines',args[0],args[1]), ('achieve_goals', goals)]
            elif args[1] == "way_point":
                return[('slow_survey',args[0],args[1]), ('remove_mines',args[0],args[1]), ('achieve_goals', goals)]
            else:
                pass
        elif predicate == "at_location" and args[0] in state.enabled:
            if args[1] == "home":
                if state.location == "transit2":
                    return[('fast_survey',args[0],args[1]), ('achieve_goals', goals)]
                else:
                    return[('fast_survey',args[0],"transit2") , ('fast_survey',args[0],args[1]), ('achieve_goals', goals)]
        elif predicate == "hazard_checked":
            vehicle = state.enabled.pop()
            return[('check',args[0],args[1],vehicle) , ('achieve_goals', goals)]
        else:
            return []
    return []


def check_hazard(state,mine,location,vehicle):
    '''
    Go through each goal and create an order for each goal
    '''

    if (location == "ga1") or (location == "ga2")or (location == "qroute") :
        return[('remove',mine,location,vehicle)]
    else:
        if mine in state.path_mines:
            return [ ('ignore',mine,location,vehicle)]

        return[('ignore',mine,location,vehicle)]

    return []

def declare_methods(longApprehend = True):
    pyhop.declare_methods("achieve_goals", achieve_goals)
    pyhop.declare_methods("check", check_hazard)
