from midca import base
from midca import midcatime
import copy

LAST_SCORED_GOAL = "Last Scored Goal"
SCORE = "Score"

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
            print("unable to check goal", currentGoal, ". skipping scoring")
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
            print("Tower", self.mem.get(MORTARSCORE).towers, "completed.", score, "added to score, which is now", self.mem.get(MORTARSCORE).score)
