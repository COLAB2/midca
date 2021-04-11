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

class DeliverGoal(base.BaseModule):

    '''
    MIDCA module that generates goals to stack blocks using Michael Maynord's TF-Trees. These trees are trained to cycle through 3 specific states; behavior is unknown for other states. See implementation in modules/_goalgen/tf_3_scen.py for details.
    '''

    def __init__(self):
        ''
    def alreadygenerated(self):
        g = self.mem.get(self.mem.DELIVERED)
        if g:
            return True

    def deliveringGoalsExist(self):
        graph = self.mem.get(self.mem.GOAL_GRAPH)
        for goal in graph.getAllGoals():
            if goal['predicate'] == "obj-at":
                return True
        return False

    def run(self, cycle, verbose = 2):
        if self.deliveringGoalsExist():
            if verbose >= 2:
                print("MIDCA already has a delivering goal. Skipping delivering goal generation")
            return

        if self.alreadygenerated():
            if verbose >= 2:
                print("MIDCA already generated the goals for this problem")
            return
        #if obj-at(p,l) is in the state, it means it needs to be delivered!
        world = self.mem.get(self.mem.STATES)[-1]

        orders = deliverstate.get_order_list(world)
#\         goal = self.tree.givegoal(blocks)
        for order in orders:
            goal = goals.Goal(order.id, order.destination, order.location, predicate = "obj-at")
            added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
            if goal:
                if verbose >= 2:
                    print("goal generated:", goal)
                ##call monitors
                m = Monitor(self.mem, "m" + order.id, order.id, goal)
#                 Thread(target=m.goalmonitor, args=[order.id, order.location, "obj-at"]).start()
                self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
