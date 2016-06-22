from MIDCA import goals, base
from MIDCA import midcatime
from _goalgen import tf_3_scen, tf_fire
from MIDCA.worldsim import blockstate
import copy

class UserGoalInput(base.BaseModule):

    '''
    MIDCA module that allows users to input goals in a predicate representation. These will be stored in MIDCA goals of the form Goal(arg1Name, arg2Name..., argiName, predicate = predName). Note that this class only allows for simple goals with only predicate and argument information. It does not currently check to see whether the type or number of arguments is appropriate.
    '''

    def parseGoal(self, txt):
        if not txt.endswith(")"):
            print "Error reading goal. Goal must be given in the form: predicate(arg1, arg2,...,argi-1,argi), where each argument is the name of an object in the world"
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
            print "Error reading goal. Goal must be given in the form: predicate(arg1, arg2,...,argi-1,argi), where each argument is the name of an object in the world"
            return None

    def objectNames(self, world):
        return world.objects.keys()

    def predicateNames(self, world):
        return world.predicates.keys()

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
            val = raw_input("Please input a goal if desired. Otherwise, press enter to continue\n")
            if not val:
                return "continue"
            elif val == 'q':
                return val
            goal = self.parseGoal(val.strip())
            if goal:
                world = self.mem.get(self.mem.STATES)[-1]
                if not self.validGoal(goal, world):
                    print str(goal), "is not a valid goal\nPossible predicates:", self.predicateNames(world), "\nPossible arguments", self.objectNames(world)
                else:
                    self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                    print "Goal added."
                    goals_entered.append(goal)

        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("USER GOALS", goals_entered)
            trace.add_data("GOAL GRAPH", copy.deepcopy(self.mem.GOAL_GRAPH))

class SimpleMortarGoalGen(base.BaseModule):
    '''
    MIDCA module that cycles through goals for the agent to achieve.
    '''
    
    curr_goal_index = 0

    curr_goal_sets = [
                  [goals.Goal(*['A_','B_'], predicate = 'stable-on'),
                   goals.Goal(*['C_','A_'], predicate = 'stable-on'),
                   goals.Goal(*['D_','C_'], predicate = 'stable-on')],
                  [goals.Goal(*['D_','B_'], predicate = 'stable-on'),
                   goals.Goal(*['B_','A_'], predicate = 'stable-on'),
                   goals.Goal(*['A_','C_'], predicate = 'stable-on')]]

    # starting state: on(D,B), on(B,A), ontable(A) ontable(C)
    # first goal: on(C,B)
    # second goal

    def next_goal(self):
        # get the next goal
        curr_goal_set = self.curr_goal_sets[self.curr_goal_index]
        # update index for next time around
        if self.curr_goal_index == len(self.curr_goal_sets)-1:
            self.curr_goal_index = 0
        else:
            self.curr_goal_index+=1
        return curr_goal_set

    def run(self, cycle, verbose=2):
        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            
        # first, check to see if we need a new goal, and only then insert a new one
        if len(self.mem.get(self.mem.GOAL_GRAPH).getAllGoals()) == 0:
            # get the next goal
            goal_set = self.next_goal()
            # insert that goal
            for g in goal_set:
                self.mem.get(self.mem.GOAL_GRAPH).insert(g)
            # update trace
            trace.add_data("NEXT GOAL(s)", goal_set)
            trace.add_data("GOAL GRAPH", copy.deepcopy(self.mem.GOAL_GRAPH))
        else:
            trace.add_data("NEXT GOAL", 'goals not empty; no goal chosen')
            trace.add_data("GOAL GRAPH", copy.deepcopy(self.mem.GOAL_GRAPH))
        


class TFStack(base.BaseModule):

    '''
    MIDCA module that generates goals to stack blocks using Michael Maynord's TF-Trees. These trees are trained to cycle through 3 specific states; behavior is unknown for other states. See implementation in modules/_goalgen/tf_3_scen.py for details.
    '''

    def __init__(self):
        self.tree = tf_3_scen.Tree()

    def stackingGoalsExist(self):
        graph = self.mem.get(self.mem.GOAL_GRAPH)
        for goal in graph.getAllGoals():
            if goal['predicate'] == "on":
                return True
        return False

    def run(self, cycle, verbose = 2):
        if self.stackingGoalsExist():
            if verbose >= 2:
                print "MIDCA already has a block stacking goal. Skipping TF-Tree stacking goal generation"
            return
        world = self.mem.get(self.mem.STATES)[-1]
        blocks = blockstate.get_block_list(world)
        goal = self.tree.givegoal(blocks)
        if goal:
            if verbose >= 2:
                print "TF-Tree goal generated:", goal
            self.mem.get(self.mem.GOAL_GRAPH).insert(goal)

class TFFire(base.BaseModule):

    '''
    MIDCA module that generates goals to put out fires using Michael Maynord's TF-Trees. The behavior is as follows: if any fires exist, a single goal will be generated to put out a fire on some block that is currently burning. Otherwise no goal will be generated.
    '''

    def __init__(self):
        self.tree = tf_fire.Tree()

    def fireGoalExists(self):
        graph = self.mem.get(self.mem.GOAL_GRAPH)
        for goal in graph.getAllGoals():
            if goal['predicate'] == "onfire":
                return True
        return False

    def run(self, cycle, verbose = 2):
        world = self.mem.get(self.mem.STATES)[-1]
        blocks = blockstate.get_block_list(world)
        goal = self.tree.givegoal(blocks)
        if goal:
            inserted = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
            if verbose >= 2:
                print "TF-Tree goal generated:", goal,
                if inserted:
                    print
                else:
                    print ". This goal was already in the graph."

class ReactiveApprehend(base.BaseModule):

    '''
    MIDCA module that generates a goal to apprehend an arsonist if there is one who is free and there is a fire in the current world state. This is designed to simulate the behavior of the Meta-AQUA system.
    '''

    def free_arsonist(self):
        world = self.mem.get(self.mem.STATES)[-1]
        for atom in world.atoms:
            if atom.predicate.name == "free" and atom.args[0].type.name == "ARSONIST":
                return atom.args[0].name
        return False

    def is_fire(self):
        world = self.mem.get(self.mem.STATES)[-1]
        for atom in world.atoms:
            if atom.predicate.name == "onfire":
                return True
        return False

    def run(self, cycle, verbose = 2):
        arsonist = self.free_arsonist()
        if arsonist and self.is_fire():
            goal = goals.Goal(arsonist, predicate = "free", negate = True)
            inserted = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
            if verbose >= 2:
                print "Meta-AQUA simulation goal generated:", goal,
                if inserted:
                    print
                else:
                    print ". This goal was already in the graph."

class InstructionReceiver:

    def init(self, world, mem):
        self.mem = mem
        self.lastTime = midcatime.now()

    def run(self, cycle, verbose = 2):
        world = self.mem.get(self.mem.STATE)
        i = len(world.utterances)
        while i > 0:
            if self.lastTime - world.utterances[i - 1].time > 0:
                break
            i -= 1
        newUtterances = [utterance.utterance for utterance in world.utterances[i:]]
        #now add goals based on new utterances
        for utterance in newUtterances:
            if verbose >= 2:
                print "received utterance:", utterance
            if utterance == "point to the quad" or utterance == "point at max":
                goal = goals.Goal(objective = "show-loc", subject = "self",
                directObject = "quad", indirectObject = "observer")
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print "adding goal:", str(goal)
                    else:
                        print "generated goal:", str(goal), "but it is already in the \
                        goal graph"
        self.lastTime = midcatime.now()
