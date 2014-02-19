from Leaf_001 import Leaf1
from Leaf_002 import Leaf2
from Internal_Node_001 import InternalNode1

class Tree():
    def __init__(self):
        self.leaf1 = Leaf1()
        self.leaf2 = Leaf2()
        self.internal1 = InternalNode1(None, self.leaf1, self.leaf2)

    def givegoal(self, blockset):
        return self.internal1.evaluate(blockset)
