"""
Blocks World methods for Pyhop 1.1.
Author: Dana Nau <nau@cs.umd.edu>, November 15, 2012
This file should work correctly in both Python 2.7 and Python 3.2.
"""

from _io import open
import time
from itertools import chain


class Monitor:

    def __init__(self, mem, name, item, goal, parent=None):
        self.name = name
        self.mem = mem
        self.block = item
        self.is_active = True
        self.is_fired = False
        self.goalmonitor = self.monitor_state
        self.goal = goal
        self.parent = parent

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
        world = self.mem.get(self.mem.STATES)[-1]

        print(('goal monitor to check ' + id + ' in' + location + ' is running... '))

        while True:
            world = self.mem.get(self.mem.STATES)[-1]
            flag = 0
            obj_exist = False
            cheatcount = cheatcount + 1
            current_goal = self.mem.get(self.mem.CURRENT_GOALS)
            current_atom = [a for a in world.atoms if a.predicate.name == predicate and a.args[0].name == id]

            # it assumes there is obj around, the monitor fires if it observes one
            if location == "unknown" and current_atom:
                obj_exist = True
                flag = -1
                if current_goal == self.goal:
                    self.goal.kwargs["probability"] = 1
                else:
                    self.goal.kwargs["probability"] = 1
                    self.mem.set(self.mem.CURRENT_GOALS, [])

            elif location == "unknown" and not current_atom:
                obj_exist = False
                if current_goal == self.goal:
                    self.goal.kwargs["probability"] = 0
                    self.mem.set(self.mem.CURRENT_GOALS, [])
                else:
                    self.goal.kwargs["probability"] = 0

            elif location != "unknown" and current_atom:
                obj_exist = True

            elif location != "unknown" and not current_atom:
                obj_exist = True
                self.mem.get(self.mem.GOAL_GRAPH).remove(self.goal)

            time.sleep(3)