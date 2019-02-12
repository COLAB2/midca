'''
A collection of functions that are domain specific, which different MIDCA components use
'''
import os,copy
from midca.modules._plan import pyhop

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures

def preferApprehend(goal1, goal2):
    if 'predicate' not in goal1 or 'predicate' not in goal2:
        return 0
    elif goal1['predicate'] == 'checked_disagreement' and goal2['predicate'] != 'checked_disagreement':
        return -1
    elif goal1['predicate'] != 'checked_disagreement' and goal2['predicate'] == 'checked_disagreement':
        return 1

    return 0


def get_World(world):
    s = "[\n"
    for name in sorted(world.objects.keys()):
        object = world.objects[name]
        if not (object.type.name == "SCORE" or
                object.type.name == "EMPLOYEES" or
                object.type.name == "CUSTOMERS" or
                object.type.name == "RESOURCES"):
            s += object.name + " (" + object.type.name + ") : "
            for atom in world.atoms:
                if object in atom.args:
                    s += str(atom) + " and "
            if s[-2:] == "d ":
                s = s[:-5] + "\n"
            else:
                s = s[:-3] + "\n"
    return s + "]\n"

def display(world):
    print (get_World(world))


def pyhop_state_from_world(world, name = "state"):
    '''

    :param world: The states of the world
    :param name:
    :return: Returns the variablized state for pyhop
    '''
    s = pyhop.State(name)
    s.score = 0
    s.resource = 0
    s.policies = []
    s.president = ""
    s.customers = []
    s.employees = []

    # these are for objects
    for objname in world.objects:
        if world.objects[objname].type.name == "PRESIDENT":
            s.president = objname
        elif world.objects[objname].type.name == "CUSTOMERS":
            s.customers.append(objname)
        elif world.objects[objname].type.name == "EMPLOYEES":
            s.employees.append(objname)
        else:
            pass

    # these are for predicates and arguments
    for atom in world.atoms:
        if atom.predicate.name == "available_resources":
            s.resource = atom.args[0].name
        if atom.predicate.name == "reputable":
            s.score = atom.args[1].name
        elif atom.predicate.name == "available_policy":
            s.policies.append(atom.args[0].name)
        else:
            pass

    return s


def pyhop_tasks_from_goals(goals, pyhopState):
    alltasks = []
    ordergoals = pyhop.Goal("goals")
    ordergoals = copy.deepcopy(goals)
    if ordergoals:
        alltasks.append(("achieve_goals", ordergoals))
    return alltasks
