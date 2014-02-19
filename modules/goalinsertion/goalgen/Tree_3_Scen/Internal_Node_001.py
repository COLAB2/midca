import random

from utils.block import Block
from Internal_Node import InternalNode

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


