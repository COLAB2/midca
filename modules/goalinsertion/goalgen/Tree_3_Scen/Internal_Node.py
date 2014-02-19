
class InternalNode:
    def __init__(self, parent=None, childyes=None, childno=None):
        self.parent = parent
        self.childyes = childyes
        self.childno = childno

    def ground(self, blockset, groundings):
        pass
