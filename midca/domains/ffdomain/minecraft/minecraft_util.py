# This file contains helpful functions for the nbeacons domain

import os


'''
 translate MIDCA goal state to problem file in metricFF 
 
'''




def ff_goals_from_midca_goals(goals, STATE_FILE):
    f = open(STATE_FILE, 'a')

    f.write(" (:goal\n")
    f.write("(and\n")
    predicate = None
    func = None
    for goal in goals:
        # extract predicate
        if 'predicate' in goal.kwargs:
            predicate = str(goal.kwargs['predicate'])

        elif 'func' in goal.kwargs:
            func = str(goal.kwargs['func'])
            val = str(goal.kwargs['val'])

        else:
            raise ValueError("Goal " + str(goal) + " does not translate")

        args = [str(arg) for arg in goal.args]
        goalargs = ' '.join(args)
        if predicate and goal['negate']:
            f.write("(not(" +predicate + " " + goalargs + "))\n")
        if predicate and not goal['negate']:
            f.write("(" + predicate + " " + goalargs + ")\n")
        elif func:
            f.write("( > (" + func + " " +goalargs + ") " + val +")\n")


    f.write(")\n")
    f.write("))")
    f.close()
