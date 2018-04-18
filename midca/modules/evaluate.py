from midca import base
from midca import midcatime
import copy
try:
   from midca.examples._gazebo_baxter import halo_color
except ImportError:
   pass


class EvalPointingFromFeedback(base.BaseModule):
    def run(self, cycle, verbose = 2):
        try:
            goals = self.mem.get(self.mem.CURRENT_GOALS)[-1]
        except:
            goals = []
        if not goals:
            if verbose >= 2:
                print "No current goals. Skipping eval"
        else:
            goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
            plan = goalGraph.getMatchingPlan(goals)
            if not plan:
                if verbose >= 2:
                    print "No plan found that achieves all current goals. ",
                    "Skipping eval based on plan completion"
            else:
                if plan.finished():
		    try:
			hallo = halo_color.HaloLed()
			# set hallo to green as a sign for monitors
			hallo.setGreen()
		    except:
			pass
	    	    finally:
			print("robot stuff")
                    if verbose >= 1:
                        print "Plan:", plan, "complete. Removing its goals"
                    for goal in plan.goals:
                        goalGraph.remove(goal)
                    numPlans = len(goalGraph.plans)
                    goalGraph.removeOldPlans()
                    newNumPlans = len(goalGraph.plans)
                    if numPlans != newNumPlans and verbose >= 1:
                        print "removing", numPlans - newNumPlans,
                        "plans that no longer apply."
		    del goals[-1]
	    	    self.mem.set(self.mem.CURRENT_GOALS,goals)
                else:
                    if verbose >= 2:
                        print "Plan:", plan, "not complete"

class SimpleEval(base.BaseModule):

    def run(self, cycle, verbose = 2):
        world = self.mem.get(self.mem.STATES)[-1]
        try:
            goals = self.mem.get(self.mem.CURRENT_GOALS)[-1]
        except:
            goals = []

        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("WORLD", copy.deepcopy(world))
            trace.add_data("GOALS", copy.deepcopy(goals))

        goals_changed = False # for trace
        if goals:
            for goal in goals:
                try:
                    achieved = world.atom_true(world.midcaGoalAsAtom(goal))
                    if 'negate' in goal and goal['negate']:
                        achieved = not achieved
                    if not achieved:
                        if verbose >= 2:
                            print "Not all goals achieved;", goal, "is not true."
                        return
                except ValueError:
                    if verbose >= 1:
                        print "Could not test goal", goal, ". It does not seem to be a valid world state"
                    return
            if verbose >= 1:
                print "All current goals achieved. Removing them from goal graph"
            goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
            for goal in goals:
                goalGraph.remove(goal)
                if trace: trace.add_data("REMOVED GOAL", goal)
                goals_changed = True
            numPlans = len(goalGraph.plans)
            goalGraph.removeOldPlans()
            newNumPlans = len(goalGraph.plans)
            if numPlans != newNumPlans and verbose >= 1:
                print "removing", numPlans - newNumPlans, "plans that no longer apply."
                goals_changed = True
	    del goals[-1]
	    self.mem.set(self.mem.CURRENT_GOALS,goals)
        else:
            if verbose >= 2:
                print "No current goals. Skipping eval"

        if trace and goals_changed: trace.add_data("GOALS",goals)


class SimpleEval2(base.BaseModule):

    def run(self, cycle, verbose = 2):
        world = self.mem.get(self.mem.STATES)[-1]
        try:
            goals = self.mem.get(self.mem.CURRENT_GOALS)[-1]
        except:
            goals = []

        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("WORLD", copy.deepcopy(world))
            trace.add_data("GOALS", copy.deepcopy(goals))

        goals_changed = False # for trace
        if goals:
            for goal in goals:
                try:
                    achieved = world.atom_true(world.midcaGoalAsAtom(goal))
                    if 'negate' in goal and goal['negate']:
                        achieved = not achieved
                    if achieved:
                        print "achieved"
                        score = self.mem.get(self.mem.DELIVERED)
                        lastGoals = self.mem.get(LAST_SCORED_GOAL)
                        if not lastGoals:
                            self.mem.set(LAST_SCORED_GOAL, [goal])
                            self.mem.set(self.mem.DELIVERED, 1)
#                             self.generate_thief_file(goal, 5)
                        elif not (goal in lastGoals):
                            self.mem.add(LAST_SCORED_GOAL, goal)
                            self.mem.set(self.mem.DELIVERED, score+1)
                        
                        
                        print str(self.mem.get(self.mem.DELIVERED))    
                    if not achieved:
                        if verbose >= 2:
                            print "Not all goals achieved;", goal, "is not true."
                        return
                except ValueError:
                    if verbose >= 1:
                        print "Could not test goal", goal, ". It does not seem to be a valid world state"
                    return
            if verbose >= 1:
                print "All current goals achieved. Removing them from goal graph"
            goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
            for goal in goals:
                goalGraph.remove(goal)
                if trace: trace.add_data("REMOVED GOAL", goal)
                goals_changed = True
            numPlans = len(goalGraph.plans)
            goalGraph.removeOldPlans()
            newNumPlans = len(goalGraph.plans)
            if numPlans != newNumPlans and verbose >= 1:
                print "removing", numPlans - newNumPlans, "plans that no longer apply."
                goals_changed = True
	    del goals[-1]
	    self.mem.set(self.mem.CURRENT_GOALS,goals)
        else:
            if verbose >= 2:
                print "No current goals. Skipping eval"

        if trace and goals_changed: trace.add_data("GOALS",goals)
    
    
    def generate_thief_file(self,goal, stolen_number):
        args = [str(arg) for arg in goal.args]
        current_w = args[2]
        graph = self.mem.get(self.mem.GOAL_GRAPH)
        goals = graph.getAllGoals()
        
        other_w_goals = []
        
        for g in goals:
            args = [str(arg) for arg in g.args]
            if args[2] != current_w:
                other_w_goals.append(g)
#         other_w_goals =  filter(lambda g: str(g.args[2]) != current_w, goals.copy())
        randomgoals = random.sample(other_w_goals, stolen_number)
        
#         thisDir =  "C:/Users/Zohreh/git/midca/modules/_plan/jShop"
        thisDir = os.path.dirname(os.path.realpath(__file__))
        thief_file = thisDir + "/theif.txt"
        
        f = open(thief_file, 'w')
        
        for g in randomgoals:
            f.write("obj-at" + " " + str(g.args[0]) + " " + str(g.args[2])+"\n")
            


class SimpleEval_Restaurant(base.BaseModule):

    def run(self, cycle, verbose = 2):
        world = self.mem.get(self.mem.STATES)[-1]
        try:
            goals = self.mem.get(self.mem.CURRENT_GOALS)
        except KeyError:
            goals = []

        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("WORLD", copy.deepcopy(world))
            trace.add_data("GOALS", copy.deepcopy(goals))

        goals_changed = False # for trace

	# if the time from memory is 0 then remove all goals from the goal graph
	goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
        money = self.mem.get(self.mem.MONEY)
	if money == 0:
            print("Current Goals and Goals in Goal Graph are Removed Due to Insuffecient money")
            for goal in goalGraph.getUnrestrictedGoals():
                goalGraph.remove(goal)
            self.mem.set(self.mem.CURRENT_GOALS , [])
            self.mem.set(self.mem.GOAL_GRAPH,goalGraph)
            goals = []
	
        if goals:
            for goal in goals:
                try:
                    achieved = world.atom_true(world.midcaGoalAsAtom(goal))
                    if 'negate' in goal and goal['negate']:
                        achieved = not achieved
                    if not achieved:
                        if verbose >= 2:
                            print "Not all goals achieved;", goal, "is not true."
                        return
                except ValueError:
                    if verbose >= 1:
                        print "Could not test goal", goal, ". It does not seem to be a valid world state"
                    return
            if verbose >= 1:
                print "All current goals achieved. Removing them from goal graph"
		self.mem.set(self.mem.CURRENT_GOALS, None)
            goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
            for goal in goals:
                goalGraph.remove(goal)
                if trace: trace.add_data("REMOVED GOAL", goal)
                goals_changed = True
            numPlans = len(goalGraph.plans)
            goalGraph.removeOldPlans()
            newNumPlans = len(goalGraph.plans)
            if numPlans != newNumPlans and verbose >= 1:
                print "removing", numPlans - newNumPlans, "plans that no longer apply."
                goals_changed = True
        else:
            if verbose >= 2:
                print "No current goals. Skipping eval"

        if trace and goals_changed: trace.add_data("GOALS",goals)


class SimpleEval_construction(base.BaseModule):

    def run(self, cycle, verbose = 2):
        world = self.mem.get(self.mem.STATES)[-1]


        
        try:
            goals = self.mem.get(self.mem.CURRENT_GOALS)
        except KeyError:
            goals = []

        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("WORLD", copy.deepcopy(world))
            trace.add_data("GOALS", copy.deepcopy(goals))

	# this variable is to skip one eval phase, when the building gets completed
	try:
		self.skip
	except AttributeError:
		self.skip = False

	goals_changed = False # for trace

	goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
        time = self.mem.get(self.mem.TIME_CONSTRUCTION)
        
        if time == 0:
            print("Current Goals and Goals in Goal Graph are Removed Due to Insuffecient Time")
            for goal in goalGraph.getUnrestrictedGoals():
                goalGraph.remove(goal)
	    goalGraph.removeOldPlans()
            self.mem.set(self.mem.CURRENT_GOALS , [])
            self.mem.set(self.mem.REJECTED_GOALS , None)
            self.mem.set(self.mem.GOAL_GRAPH,goalGraph)
            goals = []

	
        rejected_goals = self.mem.get(self.mem.REJECTED_GOALS)

	if rejected_goals :
                print("Current Goals and Goals in Goal Graph are Removed Due to Insuffecient Time")
		for goal in goalGraph.getUnrestrictedGoals():
			goalGraph.remove(goal)
		self.mem.set(self.mem.CURRENT_GOALS , [])
		goals_changed = True
		self.mem.set(self.mem.REJECTED_GOALS , None)
		self.mem.set(self.mem.GOAL_GRAPH,goalGraph)
		goals=[]
	

        
        if goals:
            for goal in goals:
                try:
                    achieved = world.atom_true(world.midcaGoalAsAtom(goal))
                    if 'negate' in goal and goal['negate']:
                        achieved = not achieved
                    if not achieved:
                        if verbose >= 2:
                            print "Not all goals achieved;", goal, "is not true."
                        return
                except ValueError:
                    if verbose >= 1:
                        print "Could not test goal", goal, ". It does not seem to be a valid world state"
                    return

	    if self.skip == False:
		if verbose >= 1:
                	print "Skipping one phase"
		self.skip = True
		return
	    if self.skip ==True:
		del self.skip

            if verbose >= 1:
                print "All current goals achieved. Removing them from goal graph"
		self.mem.set(self.mem.CURRENT_GOALS, None)
            goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
            for goal in goals:
                goalGraph.remove(goal)
                if trace: trace.add_data("REMOVED GOAL", goal)
                goals_changed = True
            numPlans = len(goalGraph.plans)
            goalGraph.removeOldPlans()
            newNumPlans = len(goalGraph.plans)
            if numPlans != newNumPlans and verbose >= 1:
                print "removing", numPlans - newNumPlans, "plans that no longer apply."
                goals_changed = True
        else:
            if verbose >= 2:
                print "No current goals. Skipping eval"

        if trace and goals_changed: trace.add_data("GOALS",goals)


LAST_SCORED_GOAL = "Last Scored Goal"
SCORE = "Score"

class Score:

    def __init__(self):
        self.towers = 0
        self.score = 0

    def update(self, score):
        self.towers += 1
        self.score += score
        # print "Score just updated"

    def getTowersCompleted(self):
        return self.towers

    def getTowersScore(self):
        return self.score

    def __str__(self):
        return "towers="+str(self.towers)+",score="+str(self.score)

class Scorer:

    '''
    MIDCA module that scores MIDCA on tower construction success. Each midcatime a tower is built, MIDCA gets 1 point for each block in the tower (including the triangular one) that is not on fire.
    Note: This module must precede SimpleEval to work consistently.
    '''

    def init(self, world, mem):
        self.mem = mem
        self.world = world
        self.mem.set(LAST_SCORED_GOAL, None)
        self.mem.set(SCORE, Score())

    #returns one current block stacking goal, if one exists.
    def get_stacking_goal(self):
		try:
			if not self.mem.get(self.mem.CURRENT_GOALS)[-1]:
				return None
		except:
				return None
		for goal in self.mem.get(self.mem.CURRENT_GOALS)[-1]:
			if 'predicate' in goal and (goal['predicate'] == 'on' or goal['predicate'] == 'stable-on'):
				return goal
		return None

    def is_on_fire(self, block):
        return self.world.is_true("onfire", [block.name])

    def block_under(self, block):
        if self.world.is_true("on-table", [block.name]):
            return None
        for atom in self.world.atoms:
            if (atom.predicate.name == "on" or atom.predicate.name == "stable-on")  and atom.args[0] == block:
                return atom.args[1]
        return None

    def get_tower_score(self, goal):
        score = 0
        block = self.world.objects[goal.args[0]]
        while block:
            if not self.is_on_fire(block):
                score += 1
            block = self.block_under(block)
        return score

    def run(self, cycle, verbose = 2):
        lastGoal = self.mem.get(LAST_SCORED_GOAL)
        currentGoal = self.get_stacking_goal()
        if not currentGoal or lastGoal == currentGoal:
            return #no goal or goal already scored
        try:
            achieved = self.world.atom_true(self.world.midcaGoalAsAtom(currentGoal))
        except Exception:
            print "unable to check goal", currentGoal, ". skipping scoring"
        if 'negate' in currentGoal and currentGoal['negate']:
            achieved = not achieved
        if not achieved:
            return
        self.mem.set(LAST_SCORED_GOAL, currentGoal)
        if not self.world.is_true("triangle", [currentGoal.args[0]]):
            return #only towers with triangles on top count
        score = self.get_tower_score(currentGoal)
        self.mem.get(SCORE).update(score)
        if verbose >= 2:
            print "Tower", self.mem.get(SCORE).towers, "completed.", score, "added to score, which is now", self.mem.get(SCORE).score

MORTARSCORE = "MortarScore"

class MortarScore:

    def __init__(self):
        self.towers = 0
        self.mortarblocks = 0
        self.regularblocks = 0
        self.score = 0

    def update(self, score, mortarblocks, regularblocks):
        self.towers += 1
        self.score += score
        self.mortarblocks += mortarblocks
        self.regularblocks += regularblocks
        # print "Score just updated"

    def getTowersCompleted(self):
        return self.towers

    def getTowersScore(self):
        return self.score

    def getMortarBlocks(self):
        return self.mortarblocks
    
    def getRegularBlocks(self):
        return self.regularblocks
    
    def __str__(self):
        return "towers="+str(self.towers)+",score="+str(self.score)+",mortarblocks="+str(self.mortarblocks)+",regularblocks="+str(self.regularblocks)


class MortarScorer:

    '''
    MIDCA module that scores MIDCA on tower construction success using mortar.
    Each midcatime a tower is built, MIDCA gets 2 points for each block in the tower that used mortar, otherwise 1 point per block.
    Note: This module must precede SimpleEval to work consistently.
    '''

    def init(self, world, mem):
        self.mem = mem
        self.world = world
        self.mem.set(LAST_SCORED_GOAL, None)
        self.mem.set(MORTARSCORE, MortarScore())

    #returns one current block stacking goal, if one exists.
    def get_stacking_goal(self):
         try:
            if not self.mem.get(self.mem.CURRENT_GOALS)[-1]:
		  return None
	 except:
		  return None 
	 for goal in self.mem.get(self.mem.CURRENT_GOALS)[-1]:
            if 'predicate' in goal and goal['predicate'] == 'on':
                return goal
         return None
    
    # TODO: should there only be 1 get_stacking_goal function?
    # I added this function when the goal was multiple atoms, the previous method (above)
    # was assuming goals were only single atoms (i.e. a single on(A,B) statement).
    # which doesn't make sense if you want to specify all the blocks of a tower
    def get_all_stacking_goals(self):
        #print("self.mem.get(self.mem.CURRENT_GOALS) = "+str(self.mem.get(self.mem.CURRENT_GOALS)))
        # try-catch for making current goals as stack
        try:
            if not self.mem.get(self.mem.CURRENT_GOALS)[-1]:
                return None
        except:
            return None
        for goal in self.mem.get(self.mem.CURRENT_GOALS)[-1]:
                #print("goal.args[0] = "+str(goal.args[0]))
                if 'predicate' in goal and (goal['predicate'] == 'on' or goal['predicate'] == 'stable-on') and goal.args[0] == 'D_': # TODO this should just automatically figure out the highest block in the tower, but this is assuming D is always the highest, which in Intend, it will always choose goals with 'D' on top
                    return goal
        return None

                
    def has_mortar(self, block):
        # see if hasmortar on this block is true
        #print("self.world = "+str(self.world))
        for atom in self.world.atoms:
            #print("atom is "+str(atom))
            if atom.predicate.name == "hasmortar" and atom.args[0].name == block.name:
                #print("found hasmortar("+str(atom.args[0].name)+")")
                return True 
        return False

    def block_under(self, block):
        if self.world.is_true("on-table", [block.name]):
            return None
        for atom in self.world.atoms:
            if (atom.predicate.name == "on" or atom.predicate.name == "stable-on") and atom.args[0] == block:
                return atom.args[1]
        return None
    
    def get_tower_score(self, goal):
        score = 0
        mortarblocks = 0
        regularblocks = 0
        block = self.world.objects[goal.args[0]]
        while block:
            # every block is worth a point
            score += 1 
            # if the block also has mortar, give extra point
            if self.has_mortar(block):
                score += 1
            # count mortar and non-mortar blocks separately
            if self.has_mortar(block):
                mortarblocks += 1
            else:
                regularblocks += 1
            
            block = self.block_under(block)
            # Add code here to check if the block has mortar, and if so, give an extra point
            
        return score, mortarblocks, regularblocks

    def run(self, cycle, verbose = 2):
        lastGoal = self.mem.get(LAST_SCORED_GOAL)
        currentGoal = self.get_all_stacking_goals()
        #print("current goal is "+str(currentGoal))
        if not currentGoal or lastGoal == currentGoal:
            return #no goal or goal already scored
        try:
            achieved = self.world.atom_true(self.world.midcaGoalAsAtom(currentGoal))
        except Exception:
            print "unable to check goal", currentGoal, ". skipping scoring"
        #if 'negate' in currentGoal and currentGoal['negate']: # TODO add this back in, removed bceause don't want to think about negative goals
        #    achieved = not achieved
        if not achieved:
            return
        self.mem.set(LAST_SCORED_GOAL, currentGoal)
        if not self.world.is_true("triangle", [currentGoal.args[0]]):
            return #only towers with triangles on top count
        score, mortarblocks, regularblocks = self.get_tower_score(currentGoal)
        self.mem.get(MORTARSCORE).update(score, mortarblocks, regularblocks)
        if verbose >= 2:
            print "Tower", self.mem.get(MORTARSCORE).towers, "completed.", score, "added to score, which is now", self.mem.get(MORTARSCORE).score


NORMAL_BLOCK_VAL = 1.0
ON_FIRE_BLOCK_VAL = 0.0

class Evaluator:

    def __init__(self, restartFires = True):
        self.restartFires = restartFires

    def init(self, world, mem, memKeys):
        self.mem = mem
        self.actualWorld = world
        self.towersFinished = 0
        self.fireturns = 0
        self.score = 0
        self.memKeys = memKeys

    def num_fires(self, world):
        n = 0
        for atom in world.atoms:
            if atom.predicate.name == "onfire":
                n += 1
        return n

    def num_fires_small(self, world, blocks = ("A_", "B_", "D_")):
        return len([1 for block in blocks if world.is_true("onfire", [block])])

    def put_out_fires(self, verbose = 2):
        if verbose >= 2:
            print "putting out fires"
        self.actualWorld.atoms = [atom for atom in self.actualWorld.atoms if atom.predicate.name != "onfire"]
        world = self.mem.get(self.memKeys.MEM_STATES)[-1]
        world.atoms = [atom for atom in world.atoms if atom.predicate.name != "onfire"]

    def run(self, verbose = 2):
        world = self.mem.get(self.memKeys.MEM_STATES)[-1]
        self.fireturns += self.num_fires(world)
        currentPlan = self.mem.get(self.memKeys.MEM_CURRENT_PLAN)
        if currentPlan:
            currentgoals = currentPlan.goals
        else:
            currentgoals = None
        accomplished = True
        if currentgoals:
            if not hasattr(currentgoals, "__iter__"):
                currentgoals = [currentgoals]
            for currentgoal in currentgoals:
                accomplishedthis = False
                if currentgoal.goaltype == "on":
                    accomplishedthis = world.is_true("on", [arg.id for arg in currentgoal.goalargs])
                    if currentgoal.goalargs[0].id == "D_" and currentgoal.goalargs[1].id == "C_" and accomplishedthis:
                        pass
                    elif currentgoal.goalargs[0].id == "D_" and currentgoal.goalargs[1].id == "B_" and accomplishedthis:
                        pass
                elif currentgoal.goaltype == "apprehend":
                    accomplishedthis = not world.is_true("free", [arg for arg in currentgoal.goalargs])
                elif currentgoal.goaltype == "notonfire":
                    accomplishedthis = not world.is_true("onfire", [arg.id for arg in currentgoal.goalargs])
                elif currentgoal.goaltype == "arm-empty":
                    accomplishedthis = world.is_true("arm-empty", [arg.id for arg in currentgoal.goalargs])
                accomplished = accomplished and accomplishedthis
            if verbose >= 1:
                s = "current goals: " + "".join([str(currentgoal) + " " for currentgoal in currentgoals]) + " have "
                if not accomplished:
                    s += "not "
                s += "been accomplished."
                print s
        else:
            if verbose >= 2:
                print "No current goal. Skipping Eval."
        if world.is_true("on", ["D_", "C_"]):
            if verbose >= 2:
                print "Tall Tower completed"
            self.towersFinished += 1
            self.score += (4 - self.num_fires(world)) * NORMAL_BLOCK_VAL + self.num_fires(world) * ON_FIRE_BLOCK_VAL
            if self.restartFires:
                self.put_out_fires(verbose)
        elif world.is_true("on", ["D_", "B_"]):
            if verbose >= 2:
                print "Short Tower completed"
            self.towersFinished += 1
            self.score += (3 - self.num_fires_small(world)) * NORMAL_BLOCK_VAL + self.num_fires_small(world) * ON_FIRE_BLOCK_VAL
            if self.restartFires:
                self.put_out_fires(verbose)
        if verbose >= 2:
            print "Total towers built:", self.towersFinished
            print "Total fire turns:", self.fireturns
            print "Total score:", self.score
        self.mem._update(self.memKeys.MEM_GOAL_COMPLETED, accomplished)
        if accomplished and currentPlan:
            if verbose >= 2:
                print "Goal completed for current plan. Removing all intended goals and pending plans for this goal."
            self.mem._update(self.memKeys.MEM_OLD_PLANS, currentPlan)
            self.mem.set(self.memKeys.MEM_CURRENT_PLAN, None)
            self.mem.get(self.memKeys.MEM_PLANS).remove_goals(currentgoals)
            self.mem.get(self.memKeys.MEM_GOALS).remove_goal_set(currentgoals)


class NBeaconsDataRecorder:
    '''
    Records data in the NBeacons Domain including:
    - number of goals achieved
    - number of actions executed
    - number of times replanning happened
    '''

    def __init__(self):
        pass
    
    def init(self, world, mem):
        self.mem = mem
        self.mem.set(LAST_SCORED_GOAL, None)
        self.mem.set(self.mem.GOALS_ACTIONS_ACHIEVED, [(0,'',0)])
        self.mem.set(self.mem.ACTIONS_EXECUTED, 0)

    def get_activation_goal(self):
        activation_goals = []
        
        # safety check
        if not self.mem.get(self.mem.CURRENT_GOALS)[-1]:
            return activation_goals
        
        for goal in self.mem.get(self.mem.CURRENT_GOALS)[-1]:
            if 'predicate' in goal and goal['predicate'] == 'activated':
                activation_goals.append(goal)
        return activation_goals

    def newrun(self, cycle, verbose = 2):
    
        lastGoal = self.mem.get(LAST_SCORED_GOAL)
        currentGoal = self.get_activation_goal()
        all_achieved = True
        if not currentGoal or lastGoal == currentGoal:
            return #no goal or goal already scored
        try:
            all_achieved = True
            at_least_one_achieved = False
            for goal in currentGoal:
                print "checking goal "+str(goal)
                achieved = self.world.atom_true(self.world.midcaGoalAsAtom(goal))
                if not achieved:
                    all_achieved = False
                    break
                else:
                    at_least_one_achieved = True
        except Exception:
            print "unable to check goal", currentGoal, ". skipping scoring"
        
        if at_least_one_achieved and all_achieved:
            print("All goals "+str(map(str,currentGoal))+" were achieved")
            goal_action_pairs = self.mem.get(self.mem.GOALS_ACTIONS_ACHIEVED)
            last_pair = goal_action_pairs[-1]
            actions_executed_thus_far = self.mem.get(self.mem.ACTIONS_EXECUTED) 
            self.mem.set(self.mem.GOALS_ACTIONS_ACHIEVED, goal_action_pairs+[(last_pair[0]+1,actions_executed_thus_far)])
        
            
        if not all_achieved:
            return
        self.mem.set(LAST_SCORED_GOAL, currentGoal)

    def run(self,cycle,verbose=2):
        self.verbose = verbose
        world = self.mem.get(self.mem.STATES)[-1]
        try:
            goals = self.mem.get(self.mem.CURRENT_GOALS)[-1]
        except:
            goals = []

        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("WORLD", copy.deepcopy(world))
            trace.add_data("GOALS", copy.deepcopy(goals))

        goals_changed = False # for trace
        all_goals_achieved = True
        if goals:
            for goal in goals:
                try:
                    achieved = world.atom_true(world.midcaGoalAsAtom(goal))
                    if 'negate' in goal and goal['negate']:
                        achieved = not achieved
                    if not achieved:
                        if verbose >= 2:
                            print "Not all goals achieved;", goal, "is not true."
                        return
                except ValueError:
                    all_goals_achieved = False
                    if verbose >= 1:
                        print "Could not test goal", goal, ". It does not seem to be a valid world state"
                    return
            if verbose >= 1:
                print "All current goals achieved. Removing them from goal graph"
            
            goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
            if all_goals_achieved:
                # remove goals
                for goal in goals:
                    if 'activated' in str(goal):
                        # incrememnt goals achieved counter
                        goals_achieved = self.mem.get(self.mem.GOALS_ACHIEVED)
                        if goals_achieved is None:
                            goals_achieved = 0
                        goals_achieved +=1
                        self.mem.set(self.mem.GOALS_ACHIEVED, goals_achieved)
                        # store goal achieved and actions executed
                        goal_action_pairs = self.mem.get(self.mem.GOALS_ACTIONS_ACHIEVED)
                        actions_executed_thus_far = self.mem.get(self.mem.ACTIONS_EXECUTED)
                        if self.verbose >= 1: print " just added goals actions pair: " +str((goals_achieved,str(goal.args[0]),actions_executed_thus_far)) 
                        self.mem.set(self.mem.GOALS_ACTIONS_ACHIEVED, goal_action_pairs+[(goals_achieved,str(goal.args[0]),actions_executed_thus_far)])
            
                    goalGraph.remove(goal)
                    if trace: trace.add_data("REMOVED GOAL", goal)
                    goals_changed = True
            numPlans = len(goalGraph.plans)
            goalGraph.removeOldPlans()
            newNumPlans = len(goalGraph.plans)
            if numPlans != newNumPlans and verbose >= 1:
                if self.verbose >= 1: print "removing", numPlans - newNumPlans, "plans that no longer apply."
                goals_changed = True
	    del goals[-1]
	    self.mem.set(self.mem.CURRENT_GOALS,goals)
        else:
            if verbose >= 2:
                print "No current goals. Skipping eval"

        if trace and goals_changed: trace.add_data("GOALS",goals)


    def oldrun(self,cycle,verbose=2):
        world = self.mem.get(self.mem.STATES)[-1]
        currentPlan = self.mem.get(self.mem.CURR_PLAN)
        if currentPlan:
            currentgoals = currentPlan.goals
        else:
            currentgoals = None
        accomplished = True
        if currentgoals:
            if not hasattr(currentgoals, "__iter__"):
                currentgoals = [currentgoals]
            for currentgoal in currentgoals:
                accomplishedthis = False
                if currentgoal.goaltype == "activated":
                    accomplishedthis = world.is_true("activated", [arg.id for arg in currentgoal.goalargs])
                    print("accomplishedthis = "+str(accomplishedthis)+" for activated("+str([arg.id for arg in currentgoal.goalargs]))
                accomplished = accomplished and accomplishedthis
            
            if verbose >= 1:
                s = "current goals: " + "".join([str(currentgoal) + " " for currentgoal in currentgoals]) + " have "
                if not accomplished:
                    s += "not "
                s += "been accomplished."
                print s
        else:
            if verbose >= 2:
                print "No current goal. Skipping Eval."
#         if world.is_true("on", ["D_", "C_"]):
#             if verbose >= 2:
#                 print "Tall Tower completed"
#             self.towersFinished += 1
#             self.score += (4 - self.num_fires(world)) * NORMAL_BLOCK_VAL + self.num_fires(world) * ON_FIRE_BLOCK_VAL
#             if self.restartFires:
#                 self.put_out_fires(verbose)
#         elif world.is_true("on", ["D_", "B_"]):
#             if verbose >= 2:
#                 print "Short Tower completed"
#             self.towersFinished += 1
#             self.score += (3 - self.num_fires_small(world)) * NORMAL_BLOCK_VAL + self.num_fires_small(world) * ON_FIRE_BLOCK_VAL
#             if self.restartFires:
#                 self.put_out_fires(verbose)
#         if verbose >= 2:
#             print "Total towers built:", self.towersFinished
#             print "Total fire turns:", self.fireturns
#             print "Total score:", self.score
        #self.mem._update(self.memKeys.MEM_GOAL_COMPLETED, accomplished)
        if accomplished and currentPlan:
            self.mem.set(self.mem.GOALS_ACHIEVED, 1+self.mem.get(self.mem.GOALS_ACHIEVED))
            if verbose >= 2:
                print "Goal completed for current plan. Removing all intended goals and pending plans for this goal."
            #self.mem._update(self.memKeys.MEM_OLD_PLANS, currentPlan)
            self.mem.set(self.mem.CURR_PLAN, None)
            self.mem.get(self.mem.MEM_PLANS).remove_goals(currentgoals)
            self.mem.get(self.mem.MEM_GOALS).remove_goal_set(currentgoals)

