'''
A collection of functions that are domain specific, which different MIDCA components use
'''
import os
from midca.domains.blocksworld import scene, blockstate
from midca.modules._plan import pyhop

def asqiiDisplay(world):
    '''
    Creates an asqii representation for blocksworld.
    '''
    blocks = blockstate.get_block_list(world)
    print(str(scene.Scene(blocks)))

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

def jshop_state_from_world(world, STATE_FILE, name = "state"):
#     thisDir =  os.path.dirname(os.path.realpath(__file__))
#     MIDCA_ROOT = thisDir + "/../"
#     STATE_FILE = MIDCA_ROOT + "jshop_domains/blocks_world/bw_ran_problems_500.shp"
    try:
        f = open(STATE_FILE, 'w')
    
        f.write('\n')
        f.write('(defproblem bw-ran-5-1 blocks-normal ((arm-empty)\n')
        
        for atom in world.atoms:
            if atom.predicate.name == "clear":
                f.write("(clear " +  atom.args[0].name+ ")\n")
            elif atom.predicate.name == "on":
                f.write("(on " + atom.args[0].name + " " +  atom.args[1].name + ")\n")
            elif atom.predicate.name == "on-table":
                f.write("(on-table " +  atom.args[0].name + ")\n")
        
        f.write(")\n")
        f.close()
        
    except Exception:
        print "could not read the state file. Check the path..."
        
        
def jshop_tasks_from_goals(goals,pyhopState, STATE_FILE):
#     thisDir =  os.path.dirname(os.path.realpath(__file__))
#     MIDCA_ROOT = thisDir + "/../"
#     STATE_FILE = MIDCA_ROOT + "jshop_domains/blocks_world/bw_ran_problems_500.shp"
    try:
        f = open(STATE_FILE, 'a')
    except Exception:
        print "could not read the state file. Check the path..."
    alltasks = []
    blkgoals = pyhop.Goal("goals")
    blkgoals.pos = {}
    f.write(" ((achieve-goals ( list\n")
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
        if predicate == "on":
            f.write("(on " +  args[0] + " " +  args[1] + ")\n")
        elif predicate == 'on-table':
            f.write("(on-table" + blkgoals.pos[args[0]]+ ")\n")
        
        else:
            raise Exception("No task corresponds to predicate " + predicate)
    f.write(" ))))")
    f.close()
    
def pyhop_state_from_world(world, name = "state"):
    s = pyhop.State(name)
    s.pos = {}
    s.clear = {}
    s.holding = False
    s.fire = {}
    s.free = {}
    s.fire_ext_avail = set()
    s.holdingfireext = None
    s.hasmortar = {} # keys are 
    s.mortaravailable = {}
    mortarblocks = []
    blocks = []
    for objname in world.objects:
        if world.objects[objname].type.name == "BLOCK" and objname != "table":
            blocks.append(objname)
        elif world.objects[objname].type.name == "ARSONIST":
            s.free[objname] = False
        elif world.objects[objname].type.name == "MORTARBLOCK":
            mortarblocks.append(objname)
    for atom in world.atoms:
        if atom.predicate.name == "clear":
            s.clear[atom.args[0].name] = True
        elif atom.predicate.name == "holding":
            s.holding = atom.args[0].name
        elif atom.predicate.name == "fire-extinguisher":
            s.fire_ext_avail.add(atom.args[0].name)
        elif atom.predicate.name == "holdingextinguisher":
            s.holdingfireext = atom.args[0].name
        elif atom.predicate.name == "arm-empty":
            s.holding = False
        elif atom.predicate.name == "on":
            s.pos[atom.args[0].name] = atom.args[1].name
        elif atom.predicate.name == "on-table":
            s.pos[atom.args[0].name] = "table"
        elif atom.predicate.name == "onfire":
            s.fire[atom.args[0].name] = True
        elif atom.predicate.name == "free":
            s.free[atom.args[0].name] = True
        elif atom.predicate.name == "available":
            s.mortaravailable[atom.args[0].name] = True
        elif "hasmortar" in world.types and atom.predicate.name == "hasmortar":
            s.hasmortar[atom.args[0].name] = atom.args[1].name
    for block in blocks:
        if block not in s.clear:
            s.clear[block] = False
        if block not in s.fire:
            s.fire[block] = False
        if block not in s.pos:
            s.pos[block] = "in-arm"
        if block not in s.hasmortar.keys():
            s.hasmortar[block] = False
    
    for mblock in mortarblocks:
        if mblock not in s.mortaravailable.keys():
            s.mortaravailable[mblock] = False

    return s

#note: str(arg) must evaluate to the name of the arg in the world representation for this method to work.
# pyhopState doesn't need to be used, but pyhop_tasks_from_goals() will be passed two arguments and needs
# to have it as an arg
def pyhop_tasks_from_goals(goals, pyhopState):
    alltasks = []
    blkgoals = pyhop.Goal("goals")
    blkgoals.pos = {}
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
        if predicate == "on":
            blkgoals.pos[args[0]] = args[1]
        elif predicate == 'on-table':
            blkgoals.pos[args[0]] = 'table'
        elif predicate == "onfire" and 'negate' in goal and goal['negate'] == True:
            alltasks.append(("put_out", args[0]))
        elif predicate == "free" and 'negate' in goal and goal['negate'] == True:
            alltasks.append(("catch_arsonist", args[0]))
        else:
            raise Exception("No task corresponds to predicate " + predicate)
    if blkgoals.pos:
        alltasks.append(("move_blocks", blkgoals))
    return alltasks


def mortar_pyhop_state_from_world(world, name = "state"):
    s = pyhop.State(name)
    s.pos = {}
    s.clear = {}
    s.holding = False
    s.fire = {}
    s.free = {}
    s.fire_ext_avail = set()
    s.holdingfireext = None
    s.hasmortar = {} # keys are 
    s.mortaravailable = {}
    mortarblocks = []
    blocks = []
    for objname in world.objects:
        if world.objects[objname].type.name == "BLOCK" and objname != "table":
            blocks.append(objname)
        elif world.objects[objname].type.name == "ARSONIST":
            s.free[objname] = False
        elif world.objects[objname].type.name == "MORTARBLOCK":
            mortarblocks.append(objname)
    for atom in world.atoms:
        if atom.predicate.name == "clear":
            s.clear[atom.args[0].name] = True
        elif atom.predicate.name == "holding":
            s.holding = atom.args[0].name
        elif atom.predicate.name == "fire-extinguisher":
            s.fire_ext_avail.add(atom.args[0].name)
        elif atom.predicate.name == "holdingextinguisher":
            s.holdingfireext = atom.args[0].name
        elif atom.predicate.name == "arm-empty":
            s.holding = False
        elif atom.predicate.name == "on":
            s.pos[atom.args[0].name] = atom.args[1].name
            s.clear[atom.args[1].name] = False
        elif atom.predicate.name == "stable-on":
            s.pos[atom.args[0].name] = atom.args[1].name
            s.clear[atom.args[1].name] = False
            #s.hasmortar[atom.args[1].name] = True # redundant
        elif atom.predicate.name == "on-table":
            s.pos[atom.args[0].name] = "table"
        elif atom.predicate.name == "onfire":
            s.fire[atom.args[0].name] = True
        elif atom.predicate.name == "free":
            s.free[atom.args[0].name] = True
        elif atom.predicate.name == "available":
            s.mortaravailable[atom.args[0].name] = True
        elif atom.predicate.name == "hasmortar":
            s.hasmortar[atom.args[0].name] = atom.args[1].name
    for block in blocks:
        if block not in s.clear:
            s.clear[block] = False
        if block not in s.fire:
            s.fire[block] = False
        if block not in s.pos:
            s.pos[block] = "in-arm"
        if block not in s.hasmortar.keys():
            s.hasmortar[block] = False
    
    for mblock in mortarblocks:
        if mblock not in s.mortaravailable.keys():
            s.mortaravailable[mblock] = False

    return s

#note: str(arg) must evaluate to the name of the arg in the world representation for this method to work.
def mortar_pyhop_tasks_from_goals(goals, pyhopState):
    alltasks = []
    blkgoals = pyhop.Goal("goals")
    blkgoals.pos = {}
    blkgoals.hasmortar = {}
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
        if predicate == "on":
            blkgoals.pos[args[0]] = args[1]
        elif predicate == "stable-on":
            blkgoals.pos[args[0]] = args[1]
            blkgoals.hasmortar[args[1]] = True  
        elif predicate == 'on-table':
            blkgoals.pos[args[0]] = 'table'
        elif predicate == "onfire" and 'negate' in goal and goal['negate'] == True:
            alltasks.append(("put_out", args[0]))
        elif predicate == "free" and 'negate' in goal and goal['negate'] == True:
            alltasks.append(("catch_arsonist", args[0]))
        else:
            raise Exception("No task corresponds to predicate " + predicate)
    if blkgoals.pos:
        alltasks.append(("move_blocks", blkgoals))
    return alltasks


