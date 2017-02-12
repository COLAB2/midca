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
        print('goal monitor to check '+ id +' in' + location+' is running... ')
        cheatcount = 0
        
        while(True):
            flag = 0
            cheatcount = cheatcount + 1
             
            for atom in world.atoms:
                if atom.predicate.name == predicate and atom.args[0].name == id and  atom.args[1].name == location:
                    flag = 1
            
            #this part is only for testing
            #___________________________
            if cheatcount == 7:
                for atom in world.atoms:
                    if atom.predicate.name == predicate and atom.args[0].name == id and  atom.args[1].name == location:
                        world.atoms.remove(atom)
                        break
            #___________________________
                       
            if flag == 0:        
                print('fire, goal should be removed')
                self.mem.get(self.mem.GOAL_GRAPH).remove(self.goal)
                return
            
            time.sleep(3)
            
