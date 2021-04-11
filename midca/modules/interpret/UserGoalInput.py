from midca import goals, base
from midca import midcatime
from ._goalgen import tf_fire
from ._goalgen import tf_3_scen
from midca.domains.logistics import deliverstate
from midca.domains.blocksworld import blockstate
from midca.worldsim import stateread
import copy,csv
import random
from midca.modules.monitors import Monitor

class UserGoalInput(base.BaseModule):

    '''
    MIDCA module that allows users to input goals in a predicate representation. These will be stored in MIDCA goals of the form Goal(arg1Name, arg2Name..., argiName, predicate = predName). Note that this class only allows for simple goals with only predicate and argument information. It does not currently check to see whether the type or number of arguments is appropriate.
    '''

    def parseGoal(self, txt):
        if not txt.endswith(")"):
            print("Error reading goal. Goal must be given in the form: predicate(arg1, arg2,...,argi-1,argi), where each argument is the name of an object in the world")
            return None
        try:
            if txt.startswith('!'):
                negate = True
                txt = txt[1:]
            else:
                negate = False
            predicateName = txt[:txt.index("(")]
            args = [arg.strip() for arg in txt[txt.index("(") + 1:-1].split(",")]
            #use on-table predicate
            if predicateName == 'on' and len(args) == 2 and 'table' == args[1]:
                predicateName = 'on-table'
                args = args[:1]
            if negate:
                goal = goals.Goal(*args, predicate = predicateName, negate = True)
            else:
                goal = goals.Goal(*args, predicate = predicateName)
            return goal
        except Exception:
            print("Error reading goal. Goal must be given in the form: predicate(arg1, arg2,...,argi-1,argi), where each argument is the name of an object in the world")
            return None

    def objectNames(self, world):
        return list(world.objects.keys())

    def predicateNames(self, world):
        return list(world.predicates.keys())

    def validGoal(self, goal, world):
        try:
            for arg in goal.args:
                if arg not in self.objectNames(world):
                    return False
            return goal['predicate'] in self.predicateNames(world)
        except Exception:
            return False

    def run(self, cycle, verbose = 2):
        if verbose == 0:
            return #if skipping, no user input
        goals_entered = []
        while True:
            val = input("Please input a goal if desired. Otherwise, press enter to continue\n")
            if not val:
                return "continue"
            elif val == 'q':
                return val
            goal = self.parseGoal(val.strip())
            if goal:
                world = self.mem.get(self.mem.STATES)[-1]
                if not self.validGoal(goal, world):
                    print(str(goal), "is not a valid goal\nPossible predicates:", self.predicateNames(world), "\nPossible arguments", self.objectNames(world))
                else:
                    self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                    print("Goal added.")
                    goals_entered.append(goal)

        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("USER GOALS", goals_entered)
            trace.add_data("GOAL GRAPH", copy.deepcopy(self.mem.GOAL_GRAPH))
