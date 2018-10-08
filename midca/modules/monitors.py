"""
Blocks World methods for Pyhop 1.1.
Author: Zoherh D

"""

from _io import open
import time
import uuid
from itertools import chain


class Monitor:

    def __init__(self, mem, item, goal, type=None):
        self.name = uuid.uuid4()
        self.mem = mem
        self.obj = item
        self.is_active = True
        self.is_fired = False
        self.goalmonitor = self.monitor_state_positive
        self.neggoalmonitor = self.monitor_state_neg
        self.goal = goal
        self.type = type

    def monitor_state(self, id, location, predicate):
        world = self.mem.get(self.mem.STATES)[-1]
        #         print world
        print(('goal monitor to check ' + id + ' in' + location + ' is running... '))
        cheatcount = 0

        while (True):
            world = self.mem.get(self.mem.STATES)[-1]
            flag = 0
            cheatcount = cheatcount + 1

            current_atom = [a for a in world.atoms if a.predicate.name == predicate and a.args[0].name == id]
            if current_atom:
                flag = 1

            if flag == 0:
                self.mem.get(self.mem.GOAL_GRAPH).remove(self.goal)
                #                 goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
                #                 goals = goalGraph.getUnrestrictedGoals()
                print(('monitor fires, ' + id + ' is lost; goal should be removed(' + self.goal.args[0] + ")"))
                return

            time.sleep(3)

    # it fires if what is monitored is observed
    # type is used when it monitors something from that type not any specific object.
    def monitor_state_positive(self, id, predicate, type=None):
        world = self.mem.get(self.mem.STATES)[-1]
        #         print world
        print('goal monitor to check ' + id + ' exists is running... ')

        while True:
            world = self.mem.get(self.mem.STATES)[-1]
            current_goal = self.mem.get(self.mem.CURRENT_GOALS)
            flag = 0

            if type:
                current_atom = [a for a in world.atoms if
                                a.predicate.name == predicate and a.args[0].type.name == type.name]
            else:
                current_atom = [a for a in world.atoms if a.predicate.name == predicate and a.args[0].name == id]

            if current_atom:
                flag = 1

            if flag == 1:
                if self.goal in current_goal:
                    self.goal.kwargs["probability"] = 1
                    print(id.__str__() + " is observed, the current goal's probability is 1 now")
                    break
                else:
                    self.goal.kwargs["probability"] = 1
                    self.mem.set(self.mem.CURRENT_GOALS, [])
                    print(id.__str__() + " is observed")
                    break
                return

            time.sleep(3)

    # it fires when what it is monitored is believed to be false
    def monitor_state_neg(self, id, predicate, knowpredicate, type=None):
        world = self.mem.get(self.mem.STATES)[-1]
        #         print world
        print('goal monitor to check ' + id + ' exists is running... ')

        while True:
            world = self.mem.get(self.mem.STATES)[-1]
            current_goal = self.mem.get(self.mem.CURRENT_GOALS)

            if type:
                current_atom = [a for a in world.atoms if
                                a.predicate.name == predicate and a.args[0].type.name == type.name]
            else:
                current_atom = [a for a in world.atoms if a.predicate.name == predicate and a.args[0].name == id]

            exist_obj = None
            for a in world.atoms:
                if type:
                    if a.predicate.name == knowpredicate and a.args and a.args[0].type.name == type.name:
                        exist_obj = a
                        break
                else:
                    if a.predicate.name == knowpredicate and a.args and str(a.args[0].name) == str(id):
                        exist_obj = a
                        break

            if exist_obj and not current_atom:

                if self.goal in current_goal:
                    self.goal.kwargs["probability"] = 0
                    self.mem.set(self.mem.CURRENT_GOALS, [])
                    print(id.__str__() + " does not exist, the current goal is suspended ")
                    break
                else:
                    self.goal.kwargs["probability"] = 0
                    print(id.__str__() + " does not exist, this goal's probability is 0 now")
                    break

            time.sleep(3)

    def monitor_belief(self, id, location, predicate):
        print(('goal monitor to check ' + id.__str__() + ' in ' + location.__str__() + ' is running... '))

        while True:
            world = self.mem.get(self.mem.STATES)[-1]
            flag = 0
            obj_exist = False

            current_goal = self.mem.get(self.mem.CURRENT_GOALS)
            # current_atom = [a for a in world.atoms if a.predicate and a.predicate.name == predicate and a.args[0]
            #                 and a.args[0].name == id]
            #
            current_atom = None
            for a in world.atoms:
                if a.predicate and a.predicate.name == predicate and a.args[0] and str(a.args[0].name) == str(id):
                    current_atom = a
                    break

            # exist_obj = [a for a in world.atoms if a.predicate and a.predicate.name == "known-loc" and a.args[0]
            #              and a.args[0].name == id]
            exist_obj = None
            for a in world.atoms:
                if a.predicate and a.predicate.name == "known-loc" and a.args and str(a.args[0].name) == str(id):
                    exist_obj = a
                    break

            # if exist_obj: print(exist_obj)

            # it assumes there is obj around, the monitor fires if it observes one
            if location == "unknown" and current_atom:
                if self.goal in current_goal:
                    self.goal.kwargs["probability"] = 1
                    print(id.__str__() + " is observed, the current goal's probability is 1 now")
                    break
                else:
                    self.goal.kwargs["probability"] = 1
                    self.mem.set(self.mem.CURRENT_GOALS, [])
                    print(id.__str__() + " is observed")
                    break

            # if known-skeleton but there is no (thing-at skeleton)
            elif location == "unknown" and exist_obj and not current_atom:

                if self.goal in current_goal:
                    self.goal.kwargs["probability"] = 0
                    self.mem.set(self.mem.CURRENT_GOALS, [])
                    print(id.__str__() + " does not exist, the current goal is suspended ")
                    break
                else:
                    self.goal.kwargs["probability"] = 0
                    print(id.__str__() + " does not exist, this goal's probability is 0 now")
                    break

            # elif location != "unknown" and current_atom:
            #     obj_exist = True

            elif location != "unknown" and not current_atom:
                self.mem.get(self.mem.GOAL_GRAPH).remove(self.goal)
                print(id.__str__() + "does not exist, the goal is removed")

            time.sleep(5)
