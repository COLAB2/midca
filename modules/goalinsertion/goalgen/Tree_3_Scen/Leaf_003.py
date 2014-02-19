from Leaf import Leaf
import random

from utils.block import Block
from modules.goalinsertion.goalgen.goal import Goal

class Leaf3(Leaf):

    # Returns True if the given blockset satisfies the rule associated with scenario 3
    # goal(A,B) :- clear(B), square(B), triangle(A).
    def evaluate(self, blockset):


        # iterate me, over all groundings of variables, if one is found, break, if not, rule not satisfied
        for a in range(len(blockset)):
            for b in range(len(blockset)):
                clearB = blockset[b].clear
                squareB = blockset[b].type == Block.SQUARE
                triangleA = blockset[a].type == Block.TRIANGLE

                if clearB and squareB and triangleA:
                    return Goal(Goal.GOAL_ON, [blockset[a], blockset[b]])

#        print "blockset"
#        print blockset

#        for block in blockset:
#            print block.isclear

        return None



