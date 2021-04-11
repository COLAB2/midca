from midca import goals, base
from midca import midcatime
from ._goalgen import tf_fire
from ._goalgen import tf_3_scen
from midca.domains.logistics import deliverstate
from midca.domains.blocksworld import blockstate
from midca.worldsim import stateread
import copy,csv
import random
from midca.modules.monitors import Monitor

class TFStack(base.BaseModule):

    '''
    MIDCA module that generates goals to stack blocks using Michael Maynord's TF-Trees. These trees are trained to cycle through 3 specific states; behavior is unknown for other states. See implementation in modules/_goalgen/tf_3_scen.py for details.
    '''

    def __init__(self):
        self.tree = tf_3_scen.Tree()

    def stackingGoalsExist(self):
        graph = self.mem.get(self.mem.GOAL_GRAPH)
        for goal in graph.getAllGoals():
            if goal['predicate'] == "on":
                return True
        return False

    def run(self, cycle, verbose = 2):
        if self.stackingGoalsExist():
            if verbose >= 2:
                print("MIDCA already has a block stacking goal. Skipping TF-Tree stacking goal generation")
            return
        world = self.mem.get(self.mem.STATES)[-1]
        blocks = blockstate.get_block_list(world)
        goal = self.tree.givegoal(blocks)
        if goal:
            if verbose >= 2:
                print("TF-Tree goal generated:", goal)
            self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
