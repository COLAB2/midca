from midca.modules._plan import pyhop


def fast_survey(state,uuv,location):
    # add the order to the variable order_pending
    state.survey[uuv]=location
    return state

def slow_survey(state,uuv,location):
    # add the order to the variable order_pending
    state.survey[uuv]=location
    return state

def ignore(state,hazard,location,vehicle):
    return state

def remove(state,hazard,location,vehicle):
    return state

def avoid(state,hazard,location,vehicle):
    return state

def declare_ops():
	pyhop.declare_operators(fast_survey, slow_survey, ignore,avoid, remove)
