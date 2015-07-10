from _plan import pyhop, methods_broken, operators, methods_extinguish_broken, operators_extinguish, methods_midca, operators_midca
from MIDCA import plans, base
import collections
import copy
class PyHopPlannerBroken(base.BaseModule):

    '''
    MIDCA module that implements a python version of the SHOP hierarchical task network (HTN) planner. HTN planners require a set of user-defined methods to generate plans; these are defined in the methods python module and declared in the constructor for this class.
    Note that this module uses has several methods to translate between MIDCA's world and goal representations and those used by pyhop; these should be changed if a new domain is introduced.
    '''

    def __init__(self, extinguishers = False):
        #declares pyhop methods. This is where the planner should be given the domain information it needs.
        if extinguishers:
            methods_extinguish_broken.declare_methods()
            operators_extinguish.declare_ops()
        else:
            methods_broken.declare_methods()
            operators.declare_ops()

    #this will require a lot more error handling, but ignoring now for debugging.
    def run(self, cycle, verbose = 2):
        world = self.mem.get(self.mem.STATES)[-1]
        goals = self.mem.get(self.mem.CURRENT_GOALS)

        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("WORLD", copy.deepcopy(world))
            trace.add_data("GOALS", copy.deepcopy(goals))

        if not goals:
            if verbose >= 2:
                print "No goals received by planner. Skipping planning."
            return
        try:
            midcaPlan = self.mem.get(self.mem.GOAL_GRAPH).getMatchingPlan(goals)
        except AttributeError:
            midcaPlan = None
        if midcaPlan:
            if verbose >= 2:
                print "Old plan retrieved. Checking validity...",
            valid = world.plan_correct(midcaPlan)
            if not valid:
                midcaPlan = None
                #if plan modification is added to MIDCA, do it here.
                if verbose >= 2:
                    print "invalid."
            elif verbose >= 2:
                print "valid."
            if valid:
                if verbose >= 2:
                    print "checking to see if all goals are achieved...",
                achieved = world.plan_goals_achieved(midcaPlan)
                if verbose >= 2:
                    if len(achieved) == len(midcaPlan.goals):
                        print "yes"
                    else:
                        print "no. Goals achieved: " + str({str(goal) for goal in achieved})
                if len(achieved) != len(midcaPlan.goals):
                    midcaPlan = None #triggers replanning.

        #ensure goals is a collection to simplify things later.
        if not isinstance(goals, collections.Iterable):
            goals = [goals]

        if not midcaPlan:
            #use pyhop to generate new plan
            if verbose >= 2:
                print "Planning..."
            try:
                pyhopState = pyhop_state_from_world(world)
            except Exception:
                print "Could not generate a valid pyhop state from current world state. Skipping planning"
            try:
                pyhopTasks = pyhop_tasks_from_goals(goals)
            except Exception:
                print "Could not generate a valid pyhop task from current goal set. Skipping planning"
            try:
                pyhopPlan = pyhop.pyhop(pyhopState, pyhopTasks, verbose = 0)
            except Exception:
                pyhopPlan = None
            if not pyhopPlan and pyhopPlan != []:
                if verbose >= 1:
                    print "Planning failed for ",
                    for goal in goals:
                        print goal, " ",
                    print
                if trace: trace.add_data("PLAN", pyhopPlan)
                return
            #change from pyhop plan to MIDCA plan
            midcaPlan = plans.Plan([plans.Action(action[0], *list(action[1:])) for action in pyhopPlan], goals)

            if verbose >= 1:
                print "Planning complete."
            if verbose >= 2:
                print "Plan: ", midcaPlan
            #save new plan
            if midcaPlan != None:
                self.mem.get(self.mem.GOAL_GRAPH).addPlan(midcaPlan)
            if trace: trace.add_data("PLAN",midcaPlan)

def pyhop_state_from_world(world, name = "state"):
    s = pyhop.State(name)
    s.pos = {}
    s.clear = {}
    s.holding = False
    s.fire = {}
    s.free = {}
    s.fire_ext_avail = set()
    s.holdingfireext = None
    blocks = []
    for objname in world.objects:
        if world.objects[objname].type.name == "BLOCK" and objname != "table":
            blocks.append(objname)
        elif world.objects[objname].type.name == "ARSONIST":
            s.free[objname] = False
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
    for block in blocks:
        if block not in s.clear:
            s.clear[block] = False
        if block not in s.fire:
            s.fire[block] = False
        if block not in s.pos:
            s.pos[block] = "in-arm"
    return s

#note: str(arg) must evaluate to the name of the arg in the world representation for this method to work.
def pyhop_tasks_from_goals(goals):
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
