from midca.modules._plan import pyhop


def survey(state,uuv,location):
    # add the order to the variable order_pending
    state.survey[uuv]=location

    return state

def declare_ops():
	pyhop.declare_operators(survey)
