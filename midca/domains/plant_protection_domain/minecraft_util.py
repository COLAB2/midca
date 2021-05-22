# This file contains helpful functions for the nbeacons domain

import os
import numbers
from midca.modules._plan import pyhop

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

def preferknows(goal1, goal2):
    if 'predicate' not in goal1 or 'predicate' not in goal2:
        return 0
    elif goal1['predicate'] == 'knows' and goal2['predicate'] != 'knows':
        return -1
    elif goal1['predicate'] != 'knows' and goal2['predicate'] == 'knows':
        return 1
    return 0

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


def drawPlantScene(midcastate, rtn_str=False):
    '''
    Takes the world state MIDCA and returns a str of an ascii
    drawing of the nbeacons domain for visual consumption by a human
    '''

    # SYMBOLS
    DIRT = '.'
    BEACON_UNACTIVATED = 'b'
    BEACON_ACTIVATED = 'B'
    WOOD = 'x'
    # WOOD_ON_FIRE = '*'
    AGENT = 'a'
    AGENT_WITH_UNACTIVATED_BEACON = '*'
    AGENT_WITH_ACTIVATED_BEACON = 'A'
    AGENT_WITH_WOOD = '^'
    AGENT_WITH_FIRE = '%'
    AGENT_WITH_FLARE = '#'
    FLARE = '$'
    QUICKSAND = '~'
    AGENT_WITH_QUICKSAND = '?'

    # convert MIDCA world to PyHop State (only doing this for code re-use
    pyhopState = pyhop_state_from_world(midcastate)
    #print(pyhopState)
    # print("successfully created pyhop state from midca state")
    # initialize the map with dirt
    #dim = pyhopState.dim['dim']
    grid = []
    for c in range(5):
        row = []
        for r in range(5):
            row.append(DIRT)
        grid.append(row)
    print(asciiframestr(grid))

def asciiframestr(frame, numbered_borders=True):
    result = ""
    if numbered_borders:
        result += "   "
        for i in range(len(frame)):
            if i >= 10:
                result += str(i)[1] + " "
            else:
                result += str(i) + " "
        result += '\n'
    r = 0
    for row in frame:
        if numbered_borders:
            if r < 10:
                result += str(r) + "  "  # two spaces to make it look nicer
            else:
                result += str(r) + " "  # only one space
        for val in row:
            result += val + " "
        result += "\n"
        r += 1
    # remove trailing newline
    result = result[:len(result) - 2]
    return result

def pyhop_state_from_world(world, name="state"):
    s = pyhop.State(name)
    print(s)
    s.agents = {'curiosity': '3,3'}  # put beacons here too? and mud tiles?
    s.dim = {'dim': -1}
    s.agents = {}
    s.beaconlocs = {}  # key is beacon id (e.g. b1), val is tile str like Tx3y4
    s.beacontypes = {}  # key is beacon id (e.g. b1), val is a number representing the type
    s.activated = {}  # key is beacon id (e.g. b1), val is True if activated, False otherwise
    s.agents = {}
    s.mud = {}
    s.quicksand = []
    beacons = []
    agent = None
    for objname in world.objects:
        if world.objects[objname].type.name == "BEACON":
            beacons.append(objname)
        elif world.objects[objname].type.name == "AGENT" and not agent:  # if agent already set, means multi-agent
            agent = objname
        elif world.objects[objname].type.name == "DIM":
            s.dim['dim'] = int(objname)

    # by default, make all beacons deactivated (if they are activated, will be changed below)        
    for bcn in beacons:
        s.activated[bcn] = False

    # now process atoms
    for atom in world.atoms:
        if atom.predicate.name == "beacon-at":
            b_id = atom.args[0].name
            tile_str = atom.args[1].name
            s.beaconlocs[b_id] = tile_str
        elif atom.predicate.name == "activated":
            b_id = atom.args[0].name
            s.activated[b_id] = True
        elif atom.predicate.name == "agent-at":
            s.agents[atom.args[0].name] = convert(atom.args[0].name)
        elif atom.predicate.name == "quicksand":
            s.quicksand.append(convert(atom.args[0].name))

            # convert tile names to pyhop operators
    for (k, v) in list(s.beaconlocs.items()):
        s.beaconlocs[k] = convert(v)

    # print("at the end of nbeacons_pyhop_state_from_world:")
    # print_state(s)
    return s

def convert(midca_tile_str):
    '''
    Converts a MIDCA tile str like Tx4y6 into
    a PyHOP str 4,6
    
    (it's not that PyHOP in general uses 4,6, just what the 
    operators are expecting) 
    '''
    if midca_tile_str.startswith("Tx"):
        # replace y with a comma
        new_v = midca_tile_str.replace("y", ",")
        # trim off Tx at the beginning
        new_v = new_v[2:]
        # print("new_v is "+str(new_v))
        return new_v
    else:
        # print("hit the else")
        return midca_tile_str

