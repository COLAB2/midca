from Leaf_001 import Leaf1
from Leaf_002 import Leaf2
from Leaf_003 import Leaf3
from Internal_Node_001 import InternalNode1
from Internal_Node_002 import InternalNode2

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
