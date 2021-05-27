from midca import base
from midca import midcatime
import copy

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
            print("unable to check goal", currentGoal, ". skipping scoring")
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
            print("Tower", self.mem.get(SCORE).towers, "completed.", score, "added to score, which is now", self.mem.get(SCORE).score)
