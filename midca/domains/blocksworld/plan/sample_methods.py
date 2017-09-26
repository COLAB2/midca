'''
Sample pyhop methods file. This is used to generate plans in the chicken domain.
'''

from midca.modules._plan import pyhop

def goals_valid(state, goals):
    '''
    checks to make sure no chicken is expected to be on both sides
    '''
    left = set()
    right = set()
    for goal in goals:
        try:
            chicken = goal[0]
            predicate = goal["predicate"]
            negate = 'negate' in goal and goal['negate']
        except KeyError:
            #invalid goal
            return False
        if predicate == "onleft" and negate or predicate == "onright" and not negate:
            side = "right"
        elif predicate == "onright" and negate or predicate == "onleft" and not negate:
            side = "left"
        else:
            #invalid goal
            return False
        if side == "left":
            if chicken in right:
                #chicken supposed to be on both sides
                return False
            left.add(chicken)
        else:
            if chicken in left:
                #chicken supposed to be on both sides
                return False
            right.add(chicken)
    return True

def achieve_goals_m(state, goals):
    '''
    checks for conflicting goals. If there are none, plans for each goal in turn
    assumes goals are one of:
    onright(chicken), !onright(chicken), onleft(chicken), !onleft(chicken)
    '''
    if not goals_valid(state, goals):
        return False
    return [("achieve_chicken_goals", goals)]

def achieve_chicken_goals_m(state, goals):
    '''
    pursues goals in order.
    '''
    if goals:
        return [("achieve_goal", goals[0]), ("achieve_chicken_goals", goals[1:])]
    else:
        return []

def achieve_goal_m(state, goal):
    '''
    assumes goals are one of:
    onright(chicken), !onright(chicken), onleft(chicken), !onleft(chicken)
    '''
    chicken = goal[0]
    predicate = goal["predicate"]
    negate = "negate" in goal and goal["negate"]
    if predicate == "onleft" and negate or predicate == "onright" and not negate:
        side = "right"
    elif predicate == "onright" and negate or predicate == "onleft" and not negate:
        side = "left"
    if state.is_true("onright", [chicken]) and side == "left":
        return [("crossleft", chicken)]
    elif state.is_true("onleft", [chicken]) and side == "right":
        return [("crossright", chicken)]
    else:
        return []

def declare_methods():
    pyhop.declare_methods("achieve_goals", achieve_goals_m)
    pyhop.declare_methods("achieve_chicken_goals", achieve_chicken_goals_m)
    pyhop.declare_methods("achieve_goal", achieve_goal_m)
    
