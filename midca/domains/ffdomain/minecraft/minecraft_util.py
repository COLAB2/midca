# This file contains helpful functions for the nbeacons domain

import os
import numbers

'''
 translate MIDCA goal state to problem file in metricFF 
 
'''

def ff_state_from_midca_world(world, STATE_FILE, name="state"):
    #     thisDir =  os.path.dirname(os.path.realpath(__file__))
    #     MIDCA_ROOT = thisDir + "/../"
    #     STATE_FILE = MIDCA_ROOT + "jshop_domains/logistics/problems.shp"

    f = open(STATE_FILE, 'w')
    f.write("(define \n")
    f.write("(problem wood)\n")
    f.write("(:domain minecraft-beta)\n")
    f.write("(:objects\n")

    for obj in list(world.objects.keys()):
        f.write(obj + " - " + world.objects[obj].type.name + "\n")
    f.write(")\n")

    f.write("(:init\n")

    for atom in world.atoms:
        if atom.predicate:
            f.write("(" +  atom.predicate.name + " ")

            for a in atom.args:
                f.write(a.name + " ")

            f.write(")\n")

        if atom.func:
            f.write("(= (" +  atom.func.name + " ")

            for a in atom.args:
                f.write(a.name + " ")
            f.write(") ")
            if isinstance(atom.val, numbers.Number):
                f.write(str(atom.val) + ")\n")
            else:
                f.write("0" + ")\n")


    f.write(")\n")
    f.close()



def ff_goals_from_midca_goals(goals, STATE_FILE, verbose=2):
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
            op = str(goal.kwargs['op'])

        else:
            raise ValueError("Goal " + str(goal) + " does not translate")

        goalargs = " "
        if goal.args:
            args = [str(arg) for arg in goal.args]
            goalargs = ' '.join(args)

        if predicate and 'negate' in goal.kwargs and goal['negate']:
            f.write("(not(" +predicate + " " + goalargs + "))\n")
        if predicate and not ('negate' in goal.kwargs):
            f.write("(" + predicate + " " + goalargs + ")\n")
        elif func:
            f.write("( "+ op +" (" + func + " " +goalargs + ") " + val +")\n")


    f.write(")\n")
    f.write("))")
    f.close()

def preferSurvive(goal1, goal2):
    if 'predicate' in goal1 and not ('func' in goal2):
        if goal1['predicate'] == 'survive':
            return -1
    if 'predicate' in goal1 and 'func' in goal2:
        if goal1['predicate'] == 'survive' and goal2['func'] != "player-current-healthp":
            return -1
        if goal1['predicate'] == 'survive' and goal2['func'] == "player-current-healthp":
            return 1
    if 'predicate' in goal2 and not ('func' in goal2):
        if goal1['predicate'] == 'survive':
            return 1
    if 'predicate' in goal2 and 'func' in goal2:
        if goal1['predicate'] == 'survive' and goal2['func'] != "player-current-healthp":
            return 1
        if goal1['predicate'] == 'survive' and goal2['func'] == "player-current-healthp":
            return -1

    return 0

def preferHealth(goal1, goal2):
    if 'func' in goal1 and 'func' in goal2:
        if goal1['func'] == 'player-current-health' and goal2['func'] != 'player-current-health':
            return -1
        elif goal1['func'] != 'player-current-health' and goal2['func'] == 'player-current-health':
            return 1

    if 'predicate' in goal1 and 'func' in goal2:
        return 1

    if 'predicate' in goal2 and 'func' in goal1:
        return -1

    return 0