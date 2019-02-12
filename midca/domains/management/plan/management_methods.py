from midca.modules._plan import pyhop
import random
import sys

def achieve_goals(state,goals):
    '''
    Go through each goal and create an order for each goal
    '''
    if goals != []:
        goal = goals[0]
        #print(goal)
        predicate= goal['predicate']
        args = goal.args
        if predicate == "increased_reputation":
            return [ ('create_and_implement_policy', goal),
                     ('achieve_goals', goals[1:])
                     ]

        elif predicate == "checked_disagreement":

            '''
            #baseline
            changed_resources = "R" + str(int(float(state.resource.replace("R", "")) - float(args[2])))
            if "-" in changed_resources:
                print ("Experiment Completed")
                sys.exit()
            return [("resolve_disagreement", args[0], args[1], args[2], state.resource, changed_resources)]


            '''
            #smart
            if int(args[2]) > 35:
                create_choice_list = [0.4*float(args[2]), 0.5*(float(args[2])), 0.6*(float(args[2]))]
                change = int(float(state.resource.replace("R", "")) - random.choice(create_choice_list))
                changed_resources = "R" + str(change)
                if  change <= 0:
                    print ("Experiment Completed Due to Lack of resources")
                    sys.exit()
                    return [("resolve_disagreement", args[0], args[1], args[2], state.resource, changed_resources)]


            return [("ignore_disagreement", args[0], args[1], args[2])]
        else:
            return []
    return []


def make_policy(state, goal):
    '''

    :param state: The pyhop variable which contains state information
                  defined in the util.py for every domain
    :param goal:  The actual goal in predicate argument form
    :return: the action with arguments
    '''
    resource = state.resource
    # reduce the resources by 10%
    change = int(float(state.resource.replace("R","")) - 50)
    changed_resources = "R" + str(change)

    if  change <= 0:
        print ("Experiment Completed Due to lack of resources")
        sys.exit()
    customer = random.choice(state.customers)
    employee = random.choice(state.employees)
    policy = state.policies[-1]
    score = state.score
    return [('see_statistics', resource, customer, employee),
            ('make_policy', goal.args[0], policy, resource, changed_resources, customer, employee),
            ('inform_policy', goal.args[0], policy, employee),
            ('implement_policy', goal.args[0], policy, goal.args[1], employee)
            ]


def declare_methods(longApprehend = True):
    pyhop.declare_methods("achieve_goals", achieve_goals)
    pyhop.declare_methods("create_and_implement_policy", make_policy)