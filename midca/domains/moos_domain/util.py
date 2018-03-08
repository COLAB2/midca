'''
A collection of functions that are domain specific, which different MIDCA components use
'''
import os,copy
from midca.modules._plan import pyhop


def display(world):
    print(world)

def pyhop_state_from_world(world, name = "state"):
    s = pyhop.State(name)
    # these are the states for the domain
    s.enabled = []
    s.survey = {}

    for atom in world.atoms:
	# get the orders into the s.order_received dictionary
        if atom.predicate.name == "enabled":
            s.enabled.append(atom.args[0].name)
    return s


def pyhop_tasks_from_goals(goals, pyhopState):
    alltasks = []
    ordergoals = pyhop.Goal("goals")
    ordergoals = copy.deepcopy(goals)
    if ordergoals:
        alltasks.append(("achieve_goals", ordergoals))	
    return alltasks
