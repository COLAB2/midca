"""
Blocks World methods for Pyhop 1.1.
Author: Dana Nau <nau@cs.umd.edu>, November 15, 2012
This file should work correctly in both Python 2.7 and Python 3.2.
"""

from _io import open
import time
import uuid
import threading
from itertools import chain


class StoppableThread(threading.Thread):
    """Thread class with a stop() method. The thread itself has to check
    regularly for the stopped() condition."""

    def __init__(self):
        super(StoppableThread, self).__init__()
        self._stop_event = threading.Event()

    def run(self):
        return

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()


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

    def player_location(self):
        world = self.mem.get(self.mem.STATES)[-1]
        for atom in world.atoms:
            if atom.predicate and atom.predicate.name == "player-at":
                return atom.args[0].name

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

    def nearby_location(self, user_loc):
        world = self.mem.get(self.mem.STATES)[-1]
        locations = []
        for atom in world.atoms:
            if atom.predicate and atom.predicate.name == "connect" and atom.args[0].name == user_loc:
                locations.append(atom.args[1].name)
        return locations

    def monitor_belief(self, id, location, predicate):
        print(('goal monitor to check ' + id.__str__() + ' in ' + location.__str__() + ' is running... '))
        player_loc = self.player_location()
        adj_tiles = self.nearby_location(player_loc)

        while True:
            world = self.mem.get(self.mem.STATES)[-1]

            goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
            pending_goals = goalGraph.getUnrestrictedGoals()
            goal_in_pending_list = True
            if self.goal in pending_goals or [self.goal] in pending_goals:
                goal_in_pending_list = False

            if goal_in_pending_list is False:
                return

            current_goal = self.mem.get(self.mem.CURRENT_GOALS)
            # current_atom = [a for a in world.atoms if a.predicate and a.predicate.name == predicate and a.args[0]
            #                 and a.args[0].name == id]
            #
            current_atom = None
            for a in world.atoms:
                if a.predicate and a.predicate.name == predicate and a.args[0] and str(a.args[0].name) == str(id) \
                        and str(a.args[1].name) in adj_tiles:
                    current_atom = a
                    break

            # exist_obj = [a for a in world.atoms if a.predicate and a.predicate.name == "known-loc" and a.args[0]
            #              and a.args[0].name == id]
            exist_obj = None
            for a in world.atoms:
                if a.predicate and a.predicate.name == "known-loc" and \
                        a.args and str(a.args[0].name) == str(id) and str(a.args[1].name) == str(player_loc):
                    exist_obj = a
                    break

                # it assumes there is obj around, the monitor fires if it observes one
            if location == "unknown" and current_atom:

                if [self.goal] in current_goal or self.goal in current_goal:
                    self.goal.kwargs["probability"] = 1
                    print(id.__str__() + " is observed, the current goal's probability is 1 now")

                    return
                else:
                    self.goal.kwargs["probability"] = 1
                    self.mem.set(self.mem.CURRENT_GOALS, [])
                    world.add_fact("thing-at", [id.__str__(), player_loc])
                    self.world.add_fact("thing-at", [id.__str__(), player_loc])
                    print(id.__str__() + " is observed")
                    print("this fact is added to the belief: ")
                    print("thing-at", id.__str__(), player_loc)

                    return

            # if known-skeleton but there is no (thing-at skeleton)
            elif location == "unknown" and exist_obj and not current_atom:

                if [self.goal] in current_goal or self.goal in current_goal:
                    self.goal.kwargs["probability"] = 0
                    self.mem.set(self.mem.CURRENT_GOALS, [])
                    print(id.__str__() + " does not exist, the current goal is suspended ")

                    for atom in world.atoms:
                        if atom.predicate and atom.predicate.name == "thing-at" and atom.args[0].name == id.__str__():
                            world.remove_atom(atom)
                            break
                    for atom in self.world.atoms:
                        if atom.predicate and atom.predicate.name == "thing-at" and atom.args[0].name == id.__str__():
                            self.world.remove_atom(atom)
                            break

                    return

                else:
                    self.goal.kwargs["probability"] = 0
                    print(id.__str__() + " does not exist, this goal's probability is 0 now")
                    for atom in world.atoms:
                        if atom.predicate and atom.predicate.name == "thing-at" and atom.args[0].name == id.__str__():
                            world.remove_atom(atom)
                            break
                    for atom in self.world.atoms:
                        if atom.predicate and atom.predicate.name == "thing-at" and atom.args[0].name == id.__str__():
                            self.world.remove_atom(atom)
                            break
                    return

            # elif location != "unknown" and current_atom:
            #     obj_exist = True

            elif location != "unknown" and not current_atom:
                self.mem.get(self.mem.GOAL_GRAPH).remove(self.goal)
                print(id.__str__() + "does not exist, the goal is removed")
                return

            time.sleep(3)
