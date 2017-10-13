'''
A collection of functions that are domain specific, which different MIDCA components use
'''
import os,copy
from midca.modules._plan import pyhop


def shopping_display(world):
    print(world)

def preferApprehend(goal1, goal2):
    if 'predicate' not in goal1 or 'predicate' not in goal2:
        return 0
    elif goal1['predicate'] == 'free' and goal2['predicate'] != 'free':
        return -1
    elif goal1['predicate'] != 'free' and goal2['predicate'] == 'free':
        return 1
    elif goal1['predicate'] == 'onfire' and goal2['predicate'] != 'onfire':
        return -1
    elif goal1['predicate'] != 'onfire' and goal2['predicate'] == 'onfire':
        return 1
    return 0

def preferFire(goal1, goal2):
    if 'predicate' not in goal1 or 'predicate' not in goal2:
        return 0
    elif goal1['predicate'] == 'onfire' and goal2['predicate'] != 'onfire':
        return -1
    elif goal1['predicate'] != 'onfire' and goal2['predicate'] == 'onfire':
        return 1
    return 0




def pyhop_state_from_world_restaurant(world, name = "state"):
    s = pyhop.State(name)
    # these are the states for the domain
    s.order_received = {}
    s.order_pending = {}
    s.order_prepared = {}
    s.order_served = {}
    '''
    for atom in world.atoms:
	# get the orders into the s.order_received dictionary
        if atom.predicate.name == "order_serve":
	    # if there are multiple orders for a single person,append the dishes into a list
	    # create a list of one dish
	    # atom.args[0].name is a person name
	    # atom.args[1].name is a dish name
	    print("hi")
	    if atom.args[0].name in s.order_received:
		s.order_received[atom.args[0].name].append(atom.args[1].name)
	    else:
		s.order_received[atom.args[0].name] = list(atom.args[1].name)
    '''
    return s


def pyhop_tasks_from_goals_restaurant(goals, pyhopState):
    alltasks = []
    ordergoals = pyhop.Goal("goals")
    ordergoals = copy.deepcopy(goals)
    if ordergoals:
        alltasks.append(("achieve_goals", ordergoals))	
    return alltasks
