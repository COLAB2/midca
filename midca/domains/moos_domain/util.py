'''
A collection of functions that are domain specific, which different MIDCA components use
'''
import os,copy
from midca.modules._plan import pyhop

def preferApprehend(goal1, goal2):
    if 'predicate' not in goal1 or 'predicate' not in goal2:
        return 0
    elif goal1['predicate'] == 'hazard_checked' and goal2['predicate'] != 'hazard_checked':
        return -1
    elif goal1['predicate'] != 'hazard_checked' and goal2['predicate'] == 'hazard_checked':
        return 1

    return 0

def display(world):
    print(world)

def pyhop_state_from_world(world, name = "state"):
    s = pyhop.State(name)
    # these are the states for the domain
    s.enabled = []
    s.checked_hazards = []
    s.survey = {}
    s.location = ""
    s.path_mines = []

    for atom in world.atoms:
	# get the orders into the s.order_received dictionary
        if atom.predicate.name == "enabled":
            s.enabled.append(atom.args[0].name)
        if atom.predicate.name == "hazard_checked":
            s.checked_hazards.append(atom.args[0].name)
        if atom.predicate.name == "at_location":
            s.location = atom.args[1].name
        if atom.predicate.name == "hazard_at_pathway":
            s.path_mines.append(atom.args[0].name)
    return s


def pyhop_tasks_from_goals(goals, pyhopState):
    alltasks = []
    ordergoals = pyhop.Goal("goals")
    ordergoals = copy.deepcopy(goals)
    if ordergoals:
        alltasks.append(("achieve_goals", ordergoals))	
    return alltasks
