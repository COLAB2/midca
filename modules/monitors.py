"""
Blocks World methods for Pyhop 1.1.
Author: Dana Nau <nau@cs.umd.edu>, November 15, 2012
This file should work correctly in both Python 2.7 and Python 3.2.
"""

from _io import open
import time
from itertools import chain


class Monitor():
    
    def __init__(self, mem,  name, item, goal):
        self.name = name
        self.mem = mem
        self.block = item
        self.is_active = True
        self.is_fired = False
        self.goalmonitor = self.monitor_state
        self.goal = goal

    def monitor_state(self, id, location, predicate):
        world = self.mem.get(self.mem.STATES)[-1]
#         print world
        print('goal monitor to check '+ id +' in' + location+' is running... ')
        cheatcount = 0
        
        while(True):
            world = self.mem.get(self.mem.STATES)[-1]
            flag = 0
            cheatcount = cheatcount + 1
            
            current_atom =  filter(lambda a: a.predicate.name == predicate and a.args[0].name == id, world.atoms)
            if current_atom:
                flag = 1
                     
            if flag == 0: 
                self.mem.get(self.mem.GOAL_GRAPH).remove(self.goal)
#                 goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
#                 goals = goalGraph.getUnrestrictedGoals()       
                print('monitor fires, '+ id +' is lost; goal should be removed(' + self.goal.args[0]+")")
#                 warehouse = self.goal.args[2]
#                 for g in goals:
#                     if g.args[2] == warehouse:
#                         self.mem.get(self.mem.GOAL_GRAPH).remove(g)
                return
            
            time.sleep(3)
            
