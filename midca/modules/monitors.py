"""
Blocks World methods for Pyhop 1.1.
Author: Dana Nau <nau@cs.umd.edu>, November 15, 2012
This file should work correctly in both Python 2.7 and Python 3.2.
"""

from _io import open
import time
import uuid
from itertools import chain


class Monitor:

    def __init__(self, mem, world, item, goal, parent=None):
        self.name = uuid.uuid4()
        self.mem = mem
        self.obj = item
        self.is_active = True
        self.is_fired = False
        self.goalmonitor = self.monitor_belief
        self.goal = goal
        self.parent = parent
        self.world = world

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
                    world.add_fact("thing-at", [id.__str__()])
                    self.world.add_fact("thing-at", [id.__str__()])
                    print(id.__str__() + " is observed")
                    break

            # if known-skeleton but there is no (thing-at skeleton)
            elif location == "unknown" and exist_obj and not current_atom:

                if self.goal in current_goal:
                    self.goal.kwargs["probability"] = 0
                    self.mem.set(self.mem.CURRENT_GOALS, [])
                    print(id.__str__() + " does not exist, the current goal is suspended ")

                    for atom in world.atoms:
                        if atom.predicate and atom.predicate.name == "thing-at" and atom.args[0].name == id.__str__():
                            world.remove_atom(atom)
                            break

                    break
                else:
                    self.goal.kwargs["probability"] = 0
                    print(id.__str__() + " does not exist, this goal's probability is 0 now")
                    for atom in world.atoms:
                        if atom.predicate and atom.predicate.name == "thing-at" and atom.args[0].name == id.__str__():
                            world.remove_atom(atom)
                            break
                    break

            # elif location != "unknown" and current_atom:
            #     obj_exist = True

            elif location != "unknown" and not current_atom:
                self.mem.get(self.mem.GOAL_GRAPH).remove(self.goal)
                print(id.__str__() + "does not exist, the goal is removed")

            time.sleep(3)