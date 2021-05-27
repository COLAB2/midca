import random

from midca.domains.blocksworld.block import Block
from midca.goals import Goal

class InternalNode:
    def __init__(self, parent=None, childyes=None, childno=None):
        self.parent = parent
        self.childyes = childyes
        self.childno = childno

    def ground(self, blockset, groundings):
        pass

# class([scen_001]) :- onfire(A), !.
# % 2.0/2.0=1.0
# class([scen_002]).
# % 2.0/2.0=1.0

class InternalNode1(InternalNode):

    # returns a goal, if one can be found
    def evaluate(self, blockset):
        groundings = self.ground(blockset)

        if groundings == {}:
            return self.childno.evaluate(blockset)
        else:
            return self.childyes.evaluate(blockset)

    # Returns groundings for A, B, if they exist, that satisfy 
    # clear(-A),on(A,-B),table(B), where blocks can optionally
    # be given groundings in the groundings parameter.
    def ground(self, blockset, groundings={}):

        for a in range(1 if 'A' in groundings else len(blockset)):
            A = groundings['A'] if 'A' in groundings else blockset[a]

            if not A.onfire:
                continue
            else:
                return {'A':A}

        return {}

'''
LEAVES
'''

class Leaf:
    def __init__(self):
        pass

    def evaluate(self, blockset):
        pass

class Leaf1(Leaf):

    # Returns a Goal if the given blockset satisfies the rule associated with scenario 1
    # goal(A) :- onfire(A).
    def evaluate(self, blockset):
        # iterate me, over all groundings of variables, if one is found, break, if not, rule not satisfied
        for a in range(len(blockset)):
            onfireA = blockset[a].onfire
            if onfireA:
                return Goal(blockset[a].id, negate = True, predicate = "onfire")
        return None

class Leaf2(Leaf):

    # no goal associated with absence of fire
    def evaluate(self, blockset):
        return None

class Tree():
    def __init__(self):
        self.leaf1 = Leaf1()
        self.leaf2 = Leaf2()
        self.internal1 = InternalNode1(None, self.leaf1, self.leaf2)

    def givegoal(self, blockset):
        return self.internal1.evaluate(blockset)
