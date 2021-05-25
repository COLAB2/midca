
# Each frame has a name, and various associated roles. These roles have facet(s?) which take on values which are themselves sets of one or more frames.
class Frame:
    # relations: a dictionary, key is role, value is other frames
    def __init__(self, name, isstate=False, iscenter=False):
        self.name = name
        self.isstate = isstate
        self.iscenter = iscenter
        self.roles = {}             # to contain multiple instances of the class Role. Keys are roles' names

class Role:
    def __init__(self):
        # Looking through the example XP, there are two types of facets:
        self.facetvalue = []
        self.facetrelation = []












