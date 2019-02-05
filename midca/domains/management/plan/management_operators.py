from midca.modules._plan import pyhop


def see_statistics(state, profits, customers, employees):
    return state

def make_policy(state, org, profits, customers, employees):
    # add the order to the variable order_pending
    return state

def inform_policy(state, org, employees):
    return state

def implement_policy(state, org, profits, customers, employees):
    return state

def look_at_disagreement(state, president, org, employees):
    return state

def resolve_disagreement(state, president, org, employees):
    return state

def implement_revised_policy(state, president, org, profits, customers, employees):
    return state

def declare_ops():
	pyhop.declare_operators(see_statistics, make_policy, inform_policy, implement_policy, look_at_disagreement, resolve_disagreement, implement_revised_policy)
