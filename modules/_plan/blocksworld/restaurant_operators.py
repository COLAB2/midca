import pyhop

def take_order(state,person,dish):
    # add the order to the variable order_pending
    if person in state.order_received:
	state.order_received[person].append(dish)
    else:
	state.order_received[person] = [dish]

    if person in state.order_pending:
	state.order_pending[person].append(dish)
    else:
	state.order_pending[person] = [dish]
    return state

def prepare_order(state,person,dish):
    # remove the order from the variable order_pending
    # add dish to the variable order_prepeared
    # delete the person from variable order_pending if there are no pending dishes
    state.order_pending[person].remove(dish)
    if state.order_pending[person] == []:
	del state.order_pending[person]
    if person in state.order_prepared:
	state.order_prepared[person].append(dish)
    else:
	state.order_prepared[person] = [dish]

    #print(state.order_received)
    return state

def serve_order(state,person,dish):
    # remove the order from the variable order_prepared and order_received
    # add dish to the variable order_served
    # delete the person from variable order_prepared if all the dishes are served
    # delete the person from variable order_received if all the dishes are served
    state.order_prepared[person].remove(dish)
    state.order_received[person].remove(dish)
    if state.order_prepared[person] == []:
	del state.order_prepared[person]
    if state.order_received[person] == []:
	del state.order_received[person]
    if person in state.order_served:
	state.order_served[person].append(dish)
    else:
	state.order_served[person] = [dish]
    return state

def declare_ops():
	pyhop.declare_operators(take_order,prepare_order,serve_order)	
