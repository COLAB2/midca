"""
Blocks World methods for Pyhop 1.1.
Author: Dana Nau <nau@cs.umd.edu>, November 15, 2012
This file should work correctly in both Python 2.7 and Python 3.2.
"""


from MIDCA.modules._plan import pyhop
from _io import open
import time

class Monitor():
    
    def __init__(self, name, block, depth):
        self.name = name
        self.block = block
        self.depth = depth
        self.is_active = True
        self.is_fired = False
        self.tasks = []
        self.change_state = None
        
    def add_task(self, task_name):
        self.tasks.append(task_name)

    
    
"""
Here are some helper functions that are used in the methods' preconditions.
"""
#precondition: state.clear[b1] = true
#pickup_task
#we just generate the monitor for the first task

def change_state(state, b1):
    state.clear[b1] = False
    state.pos.update({"F_" : b1})
    return state

def on_fire(state, depth, b1, task_name):
    i = 0
    m =  filter(lambda x: x.name.__name__ == "on_fire" and x.block == b1, pyhop.generated_monitors)
    if m: 
        m[0].add_task(task_name)    
        print "monitor is already running for " + b1
    else:
        print "monitor is added for" + b1 + "to check if it is on fire"
        m = Monitor(clear_block, b1, depth)
        m.add_task(task_name)
        m.change_state = change_state
        pyhop.generated_monitors.append(m)
        status_on_fire = state.fire[b1]
        
        while(m.is_fired == False):
                        #f.write (b1 + " true")
            i = i + 1
            time.sleep(50)
            if i > 3:
                if b1 == "XX_":
                    m.is_fired = True
                    state.fire['XX_'] = False
                    print("monitor: " + b1 + "fire status changed")
                         
            if state.fire[b1] != status_on_fire:
                print("monitor: " + b1 + 'fire status changed')
                m.is_fired = True
 
def clear_block(state, depth, b1, task_name):
    i = 0
    m =  filter(lambda x: x.name.__name__ == "clear_block" and x.block == b1, pyhop.generated_monitors)
    if m: 
        m[0].add_task(task_name)    
        print "monitor is already running for " + b1
    else:
        print "monitor is added for" + b1 + "to check if it is clear"
        m = Monitor(clear_block, b1, depth)
        m.add_task(task_name)
        m.change_state = change_state
        pyhop.generated_monitors.append(m)
    
    
        while(m.is_fired == False):
                        #f.write (b1 + " true")
            i = i + 1
           
            if i > 3000:
                if b1 == "C_":
                    m.is_fired = True
                    state.clear['C_'] = False
                    state.pos.update({"D_" : 'C_'})
                    print("monitor: " + b1 + "clear status changed")
                         
            if state.clear[b1] == False:
                print("monitor: " + b1 + "is not clear!")
                m.is_fired = True
    
#         while(m.is_fired == False):
#                         #f.write (b1 + " true")
#             i = i + 1
#             time.sleep(2)
#             if i > 3:
#                 if b1 == "C_":
#                     m.is_fired = True
#                     state.clear['A'] = False
#                     state.pos.update({"B_" : 'table'})
#                     print("monitor: " + b1 + "is not clear!")
#                          
#             if state.clear[b1] == False:
#                 print("monitor: " + b1 + "is not clear!")
#                 m.is_fired = True
               
#it is about the position of block b which should be on top of c
def pos_of_block(state, depth, b, task_name):
    i = 0
    m =  filter(lambda x: x.name.__name__ == "pos_of_block" and x.block == b, pyhop.generated_monitors)
    if m: 
        m[0].add_task(task_name)    
        print "monitor is already running for " + b
    else:
        print "monitor for position was added"
        
        m = Monitor(pos_of_block, b, depth)
        m.add_task(task_name)
        m.change_state = change_state
        pyhop.generated_monitors.append(m)
        
        c = state.pos[b]
        print c
        i = 0
        while(m.is_fired == False):
            i = i + 1
            if i > 0:
                if b == "B_":
                    m.is_fired = True
                    state.clear[c] = True
                    state.pos.update({"B_" : 'C_'})
                    print("monitor: " + b + "is not on " + c + "anymore")
                    m.is_fired = True
                else:
                    break
#             if state.pos[b] != c:
#                 print("monitor: " + b + "is not on " + c + "anymore")
#                 m.is_fired = True
    
    
#
"""for each task we know what kind of monitors we should run"""
    
def declare_monitors(longApprehend = True):    
#     pyhop.declare_monitors('pickup_task',clear_block)
#     #unstack_task 
#     pyhop.declare_monitors('unstack_task', clear_block)
#     #unstack
#     pyhop.declare_monitors('unstack', clear_block)
#     state.pos[b] == c
    pyhop.declare_monitors('unstack', pos_of_block)
   #pickup
    
    pyhop.declare_monitors('pickup', clear_block)
    pyhop.declare_monitors('put_out', on_fire)
          
#     #get
    #pyhop.declare_monitors('get', clear_block)
#     
    
    