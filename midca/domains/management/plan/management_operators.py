from midca.modules._plan import pyhop


def see_statistics(state, resource, customer, employee):
    return state

def make_policy(state, org, policy, resource, changed_resource, customer, employee):
    # remove the policy from available policies
    state.policies.remove(policy)
    state.resource = changed_resource
    return state

def inform_policy(state, org, policy, employees):
    return state

def implement_policy(state, org, policy, score, employee):
    state.score = str(int(state.score) + int(score))
    return state


def resolve_disagreement(state, org, policy, score, old_resource, resource):
    state.resource = resource
    return state

def ignore_disagreement(state, org, policy, intensity):
    return state

def implement_revised_policy(state, president, org, profits, customers, employees):
    return state


def declare_ops():
	pyhop.declare_operators(see_statistics, make_policy, inform_policy, implement_policy, resolve_disagreement, ignore_disagreement) #look_at_disagreement, resolve_disagreement, implement_revised_policy)
