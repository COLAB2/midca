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


class ReactiveApprehend(base.BaseModule):

    '''
    MIDCA module that generates a goal to apprehend an arsonist if there is one who is free and there is a fire in the current world state. This is designed to simulate the behavior of the Meta-AQUA system.
    '''

    def free_arsonist(self):
        world = self.mem.get(self.mem.STATES)[-1]
        for atom in world.atoms:
            if atom.predicate.name == "free" and atom.args[0].type.name == "ARSONIST":
                return atom.args[0].name
        return False

    def is_fire(self):
        world = self.mem.get(self.mem.STATES)[-1]
        for atom in world.atoms:
            if atom.predicate.name == "onfire":
                return True
        return False

    def run(self, cycle, verbose = 2):
        arsonist = self.free_arsonist()
        if arsonist and self.is_fire():
            goal = goals.Goal(arsonist, predicate = "free", negate = True)
            inserted = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
            if verbose >= 2:
                print("Meta-AQUA simulation goal generated:", goal, end=' ')
                if inserted:
                    print()
                else:
                    print(". This goal was already in the graph.")
