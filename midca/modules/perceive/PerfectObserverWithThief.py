from midca import base
import copy, os

class PerfectObserverWithThief(base.BaseModule):

    '''
    MIDCA Module which copies a complete world state. It is designed to interact with the
    built-in MIDCA world simulator. To extend this to work with other representations,
    modify the observe method so that it returns an object representing the current known
    world state.
    '''

    def init(self, world, mem):
        base.BaseModule.init(self, mem)
        if not world:
            raise ValueError("world is None!")
        self.world = world

    #perfect observation
    def observe(self):
        return self.world.copy()

    def run(self, cycle, verbose = 2):
        world = self.observe()
        thisDir = os.path.dirname(os.path.realpath(__file__))
        thief_file = thisDir + "/theif.txt"
        theft_items=[]

#         with open(thief_file) as f:
#               lines = f.readlines()
#               for line in lines:
#                       theft_items.append(line.split(" "))
#
        if not world:
            raise Exception("World observation failed.")

#         self.mem.add(self.mem.STATES, world)

        for item in theft_items:

            for atom in world.atoms:
                if atom.predicate.name == item[0] and atom.args[0].name == item[1]:
                    world.atoms.remove(atom)
                    print(("removed:" + atom.args[0].name))
                    break

        self.mem.add(self.mem.STATES, world)

        # Memory Usage Optimization (optional, feel free to comment
        # drop old memory states if not being used
        # this should help with high memory costs
        states = self.mem.get(self.mem.STATES)
        if len(states) > 400:
            #print "trimmed off 200 old stale states"
            states = states[200:]
            self.mem.set(self.mem.STATES, states)
        # End Memory Usage Optimization

        if verbose >= 1:
            print("World observed.")

        trace = self.mem.trace
        if trace:
            trace.add_module(cycle, self.__class__.__name__)
            trace.add_data("WORLD",copy.deepcopy(world))
