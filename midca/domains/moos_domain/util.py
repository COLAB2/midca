'''
A collection of functions that are domain specific, which different MIDCA components use
'''
import os,copy
from midca.modules._plan import pyhop
from midca.modules._plan.asynch import asynch_updated_moos


def preferApprehend(goal1, goal2):
    if 'predicate' not in goal1 or 'predicate' not in goal2:
        return 0
    elif goal1['predicate'] == 'hazard-checked' and goal2['predicate'] != 'hazard-checked':
        return -1
    elif goal1['predicate'] != 'hazard-checked' and goal2['predicate'] == 'hazard-checked':
        return 1

    return 0

def display(world):
    print(world)
    pass

def monitor(world, plan, mem, plans):
    if plan:
        converttomidcaplan = lambda plan: [action.midcaAction for action in plan if not(action.status == 2)]

        actions = converttomidcaplan(plan)

        for atom in world.atoms:
            if atom.predicate.name == "hazard-at-pathway":
                if actions[0].op == "avoid":
                    return True, False
                return False, False

            elif atom.predicate.name == "hazard-at-location":
                if actions[0].op == "do-clear" and \
                        actions[0].args[1] == atom.args[1].name:
                    action = ["remove", atom.args[0].name, atom.args[1].name, "remus"]
                    midcaAction = plans.Action(action[0], *list(action[1:]))
                    # insertion index
                    index = 0
                    for i, action in enumerate(plan):
                        if action.midcaAction.op == "do-clear" and atom.args[1].name == action.midcaAction.args[1]:
                            index = i
                            break
                    plan.actions.insert(i,asynch_updated_moos.RemoveMine(mem, midcaAction))
                    return True, True

    return True, False



def jshop2_state_from_world(world, STATE_FILE, name = "state"):
    import os
    thisDir = os.path.dirname(os.path.realpath(__file__))
#     MIDCA_ROOT = thisDir + "/../"
# #     STATE_FILE = MIDCA_ROOT + "jshop_domains/logistics/problem"
#     STATE_FILE = "C:/Users/Zohreh/git/MIDCA/modules/_plan/jShop/problem"
    f = open(STATE_FILE, 'w')
    f.write('\n')
    f.write("(defproblem problem UMC (\n")
    f.write("\n")

    for atom in world.atoms:
        predicate = atom.predicate.name
        args = []
        for arg in atom.args:
            args.append(arg.name)

        arguments = ""
        # iterate through arguments and make them as strings
        for each in args:
            arguments += " " + each

        f.write("("+predicate + arguments +")\n")

    f.write(")\n")
    f.close()

def jshop2_tasks_from_goals(goals,pyhopState, STATE_FILE):
    import os
    thisDir =  os.path.dirname(os.path.realpath(__file__))
#     MIDCA_ROOT = thisDir + "/../"
# #     STATE_FILE = MIDCA_ROOT + "jshop_domains/logistics/problem"
#     STATE_FILE = "C:/Users/Zohreh/git/MIDCA/modules/_plan/jShop/problem"
    f = open(STATE_FILE, 'a')

    alltasks = []
    f.write(" ((achieve-goals (list\n")
    for goal in goals:
        #extract predicate
        if 'predicate' in goal.kwargs:
            predicate = str(goal.kwargs['predicate'])
        elif 'Predicate' in goal.kwargs:
            predicate = str(goal.kwargs['Predicate'])
        elif goal.args:
            predicate = str(goal.args[0])
        else:
            raise ValueError("Goal " + str(goal) + " does not translate to a valid pyhop task")
        args = [str(arg) for arg in goal.args]
        if args[0] == predicate:
            args.pop(0)
        arguments = " ".join(args)
        f.write("(" + predicate + " " + arguments+")\n")
    f.write(" ))))")
    f.close()

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
