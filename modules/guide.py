from MIDCA import goals, base
from MIDCA import midcatime
from _goalgen import tf_3_scen, tf_fire
from MIDCA.domains.blocksworld import blockstate
import copy 
import random

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
            if trace:
                trace.add_data("NEXT GOAL(s)", goal_set)
                trace.add_data("GOAL GRAPH", copy.deepcopy(self.mem.GOAL_GRAPH))
        else:
            if trace:
                trace.add_data("NEXT GOAL", 'goals not empty; no goal chosen')
                trace.add_data("GOAL GRAPH", copy.deepcopy(self.mem.GOAL_GRAPH))
        


class NBeaconsGoalGenerator(base.BaseModule):
    '''
    MIDCA module for the nbeacons domain. Generates a goal to activate 3 different
    beacons in the domain.
    '''
    
    def __init__(self, numbeacons=3, goalList=[]):
        self.numbeacons = numbeacons
        self.goalList = goalList
        self.currGoalIndex = 0
    
    def activateGoalsExist(self):
        graph = self.mem.get(self.mem.GOAL_GRAPH)
        for goal in graph.getAllGoals():
            if goal['predicate'] == "activated":
                return True
        return False
    
    def generate_new_goals(self):
        if self.currGoalIndex < len(self.goalList):
            curr_goal = self.goalList[self.currGoalIndex]
            #if self.verbose >= 1: print "inserting goal "+str(curr_goal)
            self.currGoalIndex+=1
            return [curr_goal]
        if self.currGoalIndex == len(self.goalList):
            print "No more goals..."
            self.currGoalIndex+=1
        return []
        
        # this is a safety check to make sure experiments are running correctly. 
        # if running manual (like running from examples/nbeacons...agentx.py remove this line
        raise Exception("randomly inserting goals, shouldn't be here when running from nbeacons_experiment_1.py")
        world = self.mem.get(self.mem.STATES)[-1]
        goal_b_ids = []
        # get all beacon ids
        unactivated_b_ids = []
        for obj in world.get_possible_objects("",""):
            # test if a beacon id
            if str(obj).startswith("B"):
                # now test to see if it's activated
                if not world.is_true('activated',[str(obj)]):
                    unactivated_b_ids.append(str(obj))
        
        if len(unactivated_b_ids) == 0:
            if self.verbose >= 1: print("All beacons are activated. No activation goals will be generated.")
            return []
                    
        num_chosen_beacons = 0
        while len(unactivated_b_ids) > 0 and num_chosen_beacons < self.numbeacons:
            b = random.choice(unactivated_b_ids)
            unactivated_b_ids.remove(b)
            goal_b_ids.append(b)
            num_chosen_beacons+=1
            
        # turn into goals
        new_goals = map(lambda x: goals.Goal(str(x), predicate = "activated"), goal_b_ids)
        
        return new_goals
    
    def run(self, cycle, verbose = 2):
        self.verbose = verbose
        if self.activateGoalsExist():
            if self.verbose >=1: print "MIDCA already has an activation goal. Skipping goal generation"
            return
        else:
            new_goals = self.generate_new_goals()
            for g in new_goals:
                self.mem.get(self.mem.GOAL_GRAPH).insert(g)
                if self.verbose >= 1: print("Inserted goal "+str(g))

class SimpleNBeaconsGoalManager(base.BaseModule):
    '''
    MIDCA module for the nbeacons domain. Using the discrepancies and explanations from
    note.DiscrepancyDetect and assess.SimpleExplain, this module will create new goals and 
    insert / remove goals from the goal graph.
    '''
    
    def run(self, cycle, verbose=2):
        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
        self.verbose = verbose
        # get discrepancies
        discrep = self.mem.get(self.mem.DISCREPANCY)
        # if discrepancy, get explanation
        if discrep and (len(discrep[0]) > 0 or len(discrep[1]) > 0):
            # first remove old 
            # go ahead and remove old plans for any goals the agent has
            # refresh the goals to trigger replanning
            goalgraph = self.mem.get(self.mem.GOAL_GRAPH)
            curr_goals = self.mem.get(self.mem.CURRENT_GOALS)
            
            if type(curr_goals) is not list:
                curr_goals = [curr_goals] 
            for goal in curr_goals:
                # get any plans associated with this goal, and remove them
                plan = goalgraph.getMatchingPlan([goal])
                if plan:
                    goalgraph.removePlan(plan)
                    if self.verbose >= 1: print "Just removed a plan for goal " +str(goal)
            
            #print "aware of actual discrepancy, retrieving explanation"
            explain_exists = self.mem.get(self.mem.EXPLANATION)
            explanation = self.mem.get(self.mem.EXPLANATION_VAL)
            if explain_exists:
                        
                # now do stuff based on explanation
                if 'stuck' in explanation:
                    # remove current goal from goal graph
                    # insert goal to become free
                    # The priority will automatically be shifted to 
                    goalgraph = self.mem.get(self.mem.GOAL_GRAPH)
                    free_goal = goals.Goal('Curiosity', predicate = "free")
                    goalgraph.insert(free_goal)
                    if self.verbose >= 1: print "Just inserted goal "+str(free_goal)
                    return
                else: #if 'wind' in explanation:
                    # do nothing for other explanations, this will just lead to replanning
                    return
            else:  
                if self.verbose >= 1: print "No explanation, old plans removed, but no goal management actions"    
                return

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
            if utterance == "point to the quad" or utterance == "goodbye baxter":
                goal = goals.Goal(objective = "show-loc", subject = "self", 
                directObject = "quad", indirectObject = "observer")
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print "adding goal:", str(goal)
                    else:
                        print "generated goal:", str(goal), "but it is already in the \
                        goal graph"
            if utterance == "get the red block":
                goal = goals.Goal(objective = "holding", subject = "self", 
                directObject = "red block", indirectObject = "observer", pos = 'red block:arm')
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print "adding goal:", str(goal)
                    else:
                        print "generated goal:", str(goal), "but it is already in the \
                        goal graph"
            
            if utterance == "get the green block":
                goal = goals.Goal(objective = "holding", subject = "self", 
                directObject = "green block", indirectObject = "observer", pos = 'green block:arm')
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print "adding goal:", str(goal)
                    else:
                        print "generated goal:", str(goal), "but it is already in the \
                        goal graph"

                        if utterance == "get the blue block":
                            goal = goals.Goal(objective = "holding", subject = "self", 
                                              directObject = "blue block", indirectObject = "observer", pos = 'blue block:arm')
                            added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print "adding goal:", str(goal)
                    else:
                        print "generated goal:", str(goal), "but it is already in the \
                        goal graph"                        
            if utterance == "put the green block on table":
                goal = goals.Goal(objective = "moving", subject = "self", 
                directObject = "green block", indirectObject = "observer", pos = 'green block:table')
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print "adding goal:", str(goal)
                    else:
                        print "generated goal:", str(goal), "but it is already in the \
                        goal graph"
                        if utterance == "put the blue block on table":
                            goal = goals.Goal(objective = "moving", subject = "self", 
                                  directObject = "blue block", indirectObject = "observer", pos = 'blue block:table')
                            added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print "adding goal:", str(goal)
                    else:
                        print "generated goal:", str(goal), "but it is already in the \
                        goal graph"
            
            if utterance == "put the red block on table":
                goal = goals.Goal(objective = "moving", subject = "self", 
                directObject = "red block", indirectObject = "observer", pos = 'red block:table')
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print "adding goal:", str(goal)
                    else:
                        print "generated goal:", str(goal), "but it is already in the \
                        goal graph"
            
            if utterance == "stack the green block on the red block":
                goal = goals.Goal(objective = "stacking", subject = "self", 
                directObject = "red block", indirectObject = "observer", pos = 'green block:red block')
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print "adding goal:", str(goal)
                    else:
                        print "generated goal:", str(goal), "but it is already in the \
                        goal graph"
            
            if utterance == "stack the blue block on the red block":
                goal = goals.Goal(objective = "stacking", subject = "self", 
                directObject = "red block", indirectObject = "observer", pos = 'blue block:red block')
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print "adding goal:", str(goal)
                    else:
                        print "generated goal:", str(goal), "but it is already in the \
                        goal graph"

                if utterance == "stack the blue block on the green block":
                    goal = goals.Goal(objective = "stacking", subject = "self", 
                                      directObject = "green block", indirectObject = "observer", pos = 'blue block:green block')
                    added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print "adding goal:", str(goal)
                    else:
                        print "generated goal:", str(goal), "but it is already in the \
                        goal graph"

                        if utterance == "stack the red block on the blue block":
                            goal = goals.Goal(objective = "stacking", subject = "self", 
                                              directObject = "blue block", indirectObject = "observer", pos = 'red block:blue block')
                            added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print "adding goal:", str(goal)
                    else:
                        print "generated goal:", str(goal), "but it is already in the \
                        goal graph"

                        if utterance == "stack the green block on the blue block":
                            goal = goals.Goal(objective = "stacking", subject = "self", 
                                              directObject = "blue block", indirectObject = "observer", pos = 'green block:blue block')
                            added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print "adding goal:", str(goal)
                    else:
                        print "generated goal:", str(goal), "but it is already in the \ goal graph"
            

            if utterance == "stack the red block on the green block":
                goal = goals.Goal(objective = "stacking", subject = "self", 
                directObject = "green block", indirectObject = "observer", pos = 'red block:green block')
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print "adding goal:", str(goal)
                    else:
                        print "generated goal:", str(goal), "but it is already in the \
                        goal graph"
            
            if utterance == "stack":
                goal = goals.Goal(objective = "stacking", subject = "self", 
                directObject = "green block", indirectObject = "observer", pos = 'red block:green block')
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print "adding goal:", str(goal)
                    else:
                        print "generated goal:", str(goal), "but it is already in the \
                        goal graph"
            
            
                
#             else:
#                 print "message is unknown"
                            
        self.lastTime = midcatime.now()
        
