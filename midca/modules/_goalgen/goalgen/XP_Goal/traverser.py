from frame import Frame
from parser import Parser
from settings import *

class Traverser:
    def __init__(self, frames, mapping):
        self.frames = frames
        self.mapping = mapping

    # Finds node within XP, for which negation may produce a reasonable goal with the returned operator
    # Returns: (frame, operator, effect) triplet if an operator, frame, and effect are found which remedies the anomaly. If none are found, returns (None, None, None)
    def traverse(self):
        # Want search in a BFS manner
        queue = []          # queue for BFS

        # Find anomaly center
        center = None
        for f in self.frames.values():
            if f.iscenter:
                center = f

        covered = []        # do not re-search nodes
        queue.append(center)

        while len(queue) > 0:
            node = queue.pop(0)
            covered.append(node)
            for role in node.roles.values():
                for rn in set(role.facetvalue + role.facetrelation):
                    if rn not in covered and rn not in queue:
                        queue.append(rn)

            if node.name in self.mapping.keys():
                for [operator, effect] in self.mapping[node.name]:
                    if self.evaluate(node, effect):
                        return (node, operator, effect)

        return (None, None, None)

    # Evaluates the result of applying effect to node within frames.
    # Returns: True for good (remedies anomaly), False for bad
    def evaluate(self, node, effect):
        if effect == OPERATOR_EFFECT_NEGATION:
            negated = [node]

            # Find nodes which depend on negated nodes, negate them.
            # Stop when either the center node is negated, or no more nodes can be negated.
            # Assumption: All edges indicate dependencies. This is invalid and will need to be updated.
            expanded = True
            while expanded:
                expaned = False
                for frame in self.frames.values():
                    if not frame in negated:
                        for role in frame.roles.values():
                            for rn in set(role.facetvalue + role.facetrelation):
                                if rn in negated and not frame in negated:
                                    if frame.iscenter:  
                                        return True
                                    negated.append(frame)
                                    expanded = True
        return False

if __name__ == "__main__":
    # get text
    f = open("../output.txt", "r")
    text = f.read()
    f.close()

    # parse text
    p = Parser()
    frames = p.makeframegraph(text)

    # create mapping
    noem = {}   # Node Operator Effect Mapping
                # Keys are node/frame names, values are lists of [operatorname, effect] pairs

    noem['CRIMINAL-VOLITIONAL-AGENT.4697'] = [['apprehend', OPERATOR_EFFECT_NEGATION]]

    # Traverse
    t = Traverser(frames, noem)
    (frame, operator, effect) = t.traverse()

    print "Frame: " + frame.name
    print "Operator: " + operator
    print "Effect: " + str(effect)

