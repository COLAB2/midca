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
            
            if cheatcount == 7:
                print('fire, goal should be removed')
                self.mem.get(self.mem.GOAL_GRAPH).remove(self.goal)
            time.sleep(3)
            

        
        
    
def clear_block(state, depth, task):
        i = 0
        b1 = task[1]
        task_name = task[0]
        m =  filter(lambda x: x.name.__name__ == "clear_block" and x.block == b1, pyhop.generated_monitors)
        if m: 
            m[0].add_task(task_name)    
            #print "monitor is already running for " + b1
        else:
            #print "monitor is added for" + b1 + "to check if it is clear"
            
            m = Monitor(clear_block, b1, depth)
            m.add_task(task_name)
            m.change_state = change_state
            pyhop.generated_monitors.append(m)
            
            
            block_on_top_of_this_block = clear_block_monitor.monitor_clear_block(b1)
            if block_on_top_of_this_block:
                print("monitor fires!!")
                m.is_fired = True 
                set_clear_status(state, b1, 'not clear')
                set_position(state, block_on_top_of_this_block, b1)
            

