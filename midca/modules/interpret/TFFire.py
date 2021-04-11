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

class TFFire(base.BaseModule):

    '''
    MIDCA module that generates goals to put out fires using Michael Maynord's TF-Trees. The behavior is as follows: if any fires exist, a single goal will be generated to put out a fire on some block that is currently burning. Otherwise no goal will be generated.
    '''

    def __init__(self):
        self.tree = tf_fire.Tree()

    def fireGoalExists(self):
        graph = self.mem.get(self.mem.GOAL_GRAPH)
        for goal in graph.getAllGoals():
            if goal['predicate'] == "onfire":
                return True
        return False

    def run(self, cycle, verbose = 2):
        world = self.mem.get(self.mem.STATES)[-1]
        blocks = blockstate.get_block_list(world)
        goal = self.tree.givegoal(blocks)
        if goal:
            inserted = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
            if verbose >= 2:
                print("TF-Tree goal generated:", goal, end=' ')
                if inserted:
                    print()
                else:
                    print(". This goal was already in the graph.")
