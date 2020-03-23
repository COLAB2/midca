'''
A collection of functions that are domain specific, which different MIDCA components use
'''
import os,copy
from midca.modules._plan import pyhop



def preferApprehend(goal1, goal2):
    if 'predicate' not in goal1 or 'predicate' not in goal2:
        return 0

    elif goal1['predicate'] == 'apprehended' and goal2['predicate'] != 'apprehended':
        return -1
    elif goal1['predicate'] != 'apprehended' and goal2['predicate'] == 'apprehended':
        return 1



    elif goal1.args[1] == "qroute1" and goal2.args[1] != "qroute1":
            return -1
    elif goal1.args[1] != "qroute1" and goal2.args[1] == "qroute1":
            return 1


    elif goal1.args[1] == "ga3" and goal2.args[1] != "ga3":
            return -1
    elif goal1.args[1] != "ga3" and goal2.args[1] == "ga3":
            return 1


    elif goal1['predicate'] == 'reported' and goal2['predicate'] != 'reported':
        return -1
    elif goal1['predicate'] != 'reported' and goal2['predicate'] == 'reported':
        return 1




    elif goal1['predicate'] == 'hazard_checked' and goal2['predicate'] != 'hazard_checked':
        return -1
    elif goal1['predicate'] != 'hazard_checked' and goal2['predicate'] == 'hazard_checked':
        return 1

    elif goal1.args[1] == "ga1" and goal2.args[1] != "ga1":
            return -1
    elif goal1.args[1] != "ga1" and goal2.args[1] == "ga1":
            return 1

    elif len(goal1.args) ==2  and len(goal2.args) == 2 :
        if goal1.args[1] == "way_point" and goal2.args[1] != "way_point":
            return -1
        elif goal1.args[1] != "way_point" and goal2.args[1] == "way_point":
            return 1

    elif len(goal1.args) ==2 and len(goal2.args) !=2:
        if goal1.args[1] == "way_point":
            return -1

    elif len(goal2.args) ==2 and len(goal1.args) !=2:
        if goal2.args[1] == "way_point":
            return 1

    return 0

def display(world):
    print(world)
    pass

def pyhop_state_from_world(world, name = "state"):
    s = pyhop.State(name)
    # these are the states for the domain
    s.enabled = []
    s.checked_hazards = []
    s.survey = {}
    s.location = ""
    s.ships = []
    s.path_mines = []
    s.passed = []

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
        if atom.predicate.name == "passed":
            s.passed.append(atom.args[1].name)
    for objname in world.objects:
        if world.objects[objname].type.name == "SHIP":
            s.ships.append(objname)
    return s


def pyhop_tasks_from_goals(goals, pyhopState):
    alltasks = []
    ordergoals = pyhop.Goal("goals")
    ordergoals = copy.deepcopy(goals)
    if ordergoals:
        alltasks.append(("achieve_goals", ordergoals))
    return alltasks

def polynomial_regression(data=0,  deg = 1):
    '''

    :param data: gets the current X value to predict y
    :param deg:  What will be the degree of polynomial equation
    :return: The new predicted Y value from the equation
    '''

    # statistical imports
    import numpy as np
    import matplotlib.pyplot as plt
    import pandas as pd
    from sklearn.linear_model import LinearRegression
    from sklearn.preprocessing import PolynomialFeatures

    datas = pd.read_csv('/home/sampath/moos-ivp/moos-ivp-midca/missions/gatars/mines_qroute.csv')
    x = []
    y = []
    # get the data (x and y rows)
    x = datas.iloc[:, 1:2].values
    y = datas.iloc[:, 2].values

    # create the instance for training polynomial regression
    poly = PolynomialFeatures(degree = deg)
    X_poly = poly.fit_transform(x)
    poly.fit(X_poly, y)

    # grab the line equation out of it
    lin2 = LinearRegression(normalize = True)
    lin2.fit(X_poly, y)

    # predict for the new data
    predict = [[data]]
    # transform the prediction to fit the model type
    predict_ = poly.fit_transform(predict)
    lin2.predict(predict_)
    return lin2.predict(predict_)[0]
