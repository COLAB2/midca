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

#    clear(-A),on(A,-B),table(B) ?              <---- this is node 1
#    +--yes: triangle(A) ?                      <---- this is node 2
#    |       +--yes: [scen_003] 1000.0 [[scen_001:0.0,scen_002:0.0,scen_003:1000.0]]
#    |       +--no:  [scen_002] 1000.0 [[scen_001:0.0,scen_002:1000.0,scen_003:0.0]]
#    +--no:  [scen_001] 1000.0 [[scen_001:1000.0,scen_002:0.0,scen_003:0.0]]

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

            if not A.clear:
                continue

            for b in range(1 if 'B' in groundings else len(blockset)):
                B = groundings['B'] if 'B' in groundings else blockset[b]

                if not (B.type == Block.TABLE and A.on == B):
                    continue
                else:
                    return {'A':A, 'B':B}

        return {}

#    clear(-A),on(A,-B),table(B) ?              <---- this is node 1
#    +--yes: triangle(A) ?                      <---- this is node 2
#    |       +--yes: [scen_003] 1000.0 [[scen_001:0.0,scen_002:0.0,scen_003:1000.0]]
#    |       +--no:  [scen_002] 1000.0 [[scen_001:0.0,scen_002:1000.0,scen_003:0.0]]
#    +--no:  [scen_001] 1000.0 [[scen_001:1000.0,scen_002:0.0,scen_003:0.0]]

class InternalNode2(InternalNode):

    # returns a goal, if one can be found
    def evaluate(self, blockset):
        groundings = self.ground(blockset)

        if groundings == {}:
            return self.childno.evaluate(blockset)
        else:
            return self.childyes.evaluate(blockset)

    # Returns groundings for A, B, if they exist, that satisfy 
    # clear(-A),on(A,-B),table(B),triangle(A) where blocks can optionally
    # be given groundings in the groundings parameter.
    def ground(self, blockset, groundings={}):

        for a in range(1 if 'A' in groundings else len(blockset)):
            A = groundings['A'] if 'A' in groundings else blockset[a]

            groundingsnew = groundings.copy()
            groundingsnew['A'] = A

            if A.type == Block.TRIANGLE:
                # try and find grounding that satisfies condition of first node
                groundingsnew = self.parent.ground(blockset, groundingsnew)
                if groundingsnew != {}:
                    return groundingsnew

        return {}

'''
LEAF NODES
'''

class Leaf:
    def __init__(self):
        pass

    def evaluate(self, blockset):
        pass

class Leaf1(Leaf):

    # Returns True if the given blockset satisfies the rule associated with scenario 1
    # goal(A,B) :- on(A,C), on(B,D), on(C,B), square(D).
    def evaluate(self, blockset):


        # iterate me, over all groundings of variables, if one is found, break, if not, rule not satisfied
        for a in range(len(blockset)):
            for b in range(len(blockset)):
                for c in range(len(blockset)):
                    for d in range(len(blockset)):
                        onAC = blockset[a].on == blockset[c]
                        onBD = blockset[b].on == blockset[d]
                        onCB = blockset[c].on == blockset[b]
                        squareD = blockset[d].type == Block.SQUARE

                        if onAC and onBD and onCB and squareD:
                            return Goal(blockset[a].id,blockset[b].id, predicate = "on")
        return None

class Leaf2(Leaf):

    # Returns True if the given blockset satisfies the rule associated with scenario 2
    # goal(A,B) :- on(A,C), on(B,D), on(D,C), A<>D.
    def evaluate(self, blockset):
        # iterate me, over all groundings of variables, if one is found, break, if not, rule not satisfied
        for a in range(len(blockset)):
            for b in range(len(blockset)):
                for c in range(len(blockset)):
                    for d in range(len(blockset)):
                        onAC = blockset[a].on == blockset[c]
                        onBD = blockset[b].on == blockset[d]
                        onDC = blockset[d].on == blockset[c]
                        AnotD = blockset[a] != blockset[d]

                        if onAC and onBD and onDC and AnotD:
                            return Goal(blockset[a].id, blockset[b].id, predicate = "on")
        return None

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
                    return Goal(blockset[a].id, blockset[b].id, predicate = "on")
        return None

class Tree():
    def __init__(self):
        self.leaf1 = Leaf1()
        self.leaf2 = Leaf2()
        self.leaf3 = Leaf3()
        self.internal2 = InternalNode2(None, self.leaf3, self.leaf2)
        self.internal1 = InternalNode1(None, self.internal2, self.leaf1)

        self.internal2.parent = self.internal1

    def givegoal(self, blockset):
        return self.internal1.evaluate(blockset)