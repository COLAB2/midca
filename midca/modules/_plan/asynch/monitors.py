"""
Blocks World methods for Pyhop 1.1.
Author: Dana Nau <nau@cs.umd.edu>, November 15, 2012
This file should work correctly in both Python 2.7 and Python 3.2.
"""


from midca.modules._plan import modified_pyhop
from _io import open
import time
from midca.vision import clear_block_monitor, pos_block_monitor
import rospy
from std_msgs.msg import String
import uuid

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



def clear_block(state, depth, task):
        i = 0
        b1 = task[1]
        task_name = task[0]
        m =  filter(lambda x: x.name.__name__ == "clear_block" and x.block == b1, modified_pyhop.generated_monitors)
        if m: 
            m[0].add_task(task_name)    
            #print "monitor is already running for " + b1
        else:
            #print "monitor is added for" + b1 + "to check if it is clear"
            
            m = Monitor(clear_block, b1, depth)
            m.add_task(task_name)
            m.change_state = change_state
            modified_pyhop.generated_monitors.append(m)
            
            
            block_on_top_of_this_block = clear_block_monitor.monitor_clear_block2(b1)
            if block_on_top_of_this_block:
                print("monitor fires!!")
                m.is_fired = True 
                set_clear_status(state, b1, 'not clear')
                set_position(state, block_on_top_of_this_block, b1)
            


def clear_block_stack(state, depth, task):
        i = 0
        c = task[2]
        b = task[1]
        #stack b on c; c should be clear
        task_name = task[0]
        print c
        m =  filter(lambda x: x.name.__name__ == "clear_block" and x.block == c, modified_pyhop.generated_monitors)
        if m: 
            m[0].add_task(task_name)    
            #print "monitor is already running for " + c
        else:
            #print "monitor is added for " + c + " to check if it is clear"
            
            m = Monitor(clear_block, c, depth)
            m.add_task(task_name)
            m.change_state = change_state
            modified_pyhop.generated_monitors.append(m)
            
            
#             rospy.loginfo("Reading point commands from topic " + unique_topic)
#         #rospy.Subscriber(targetTopic, String, fake_point) 
#             rospy.Subscriber(m.topic, String, monitor_fire_callback)    
            
            block_on_top_of_this_block = clear_block_monitor.monitor_clear_block(c)
            if block_on_top_of_this_block:
                print("monitor fires!!")
                m.is_fired = True 
                set_clear_status(state, c, 'not clear')
                set_position(state, block_on_top_of_this_block, c)
                #update the state
                #?????????????????


def set_clear_status(state, objectOrID, newClearStatus):
    
    positions = state.all_pos(objectOrID)
    if not positions:
        return None
    else:
        for state_pos in reversed(positions):
            if state_pos.isclear:
                state_pos.isclear = newClearStatus
                return state
    return None

def set_position(state, objectOrID, newPos):
    
    positions = state.all_pos(objectOrID)
    if not positions:
        return None
    else:
        for state_pos in reversed(positions):
            if state_pos.position:
                state_pos.position = newPos
                return state
    return None            
       
               
#it is about the position of block b which should be on top of c
def pos_of_block(state, depth, task):
    i = 0
    b = task[1]
    task_name = task[0]
    m =  filter(lambda x: x.name.__name__ == "pos_of_block" and x.block == b, modified_pyhop.generated_monitors)
    if m: 
        m[0].add_task(task_name)    
        #print "monitor is already running for " + b
    else:
        #print "monitor for position was added"
        
        m = Monitor(pos_of_block, b, depth)
        m.add_task(task_name)
        m.change_state = change_state
        modified_pyhop.generated_monitors.append(m)
        
        c = get_last_position(state, b)
        new_pos = pos_block_monitor.monitor_pos_block(b, c)
        if new_pos:
            print("monitor fires!!")
            m.is_fired = True
            set_position(state, b, new_pos)
            set_clear_status(state, c, 'clear')
        
        
def get_last_position(state, objectOrID):
    
    positions = state.all_pos(objectOrID)
    if not positions:
        return None
    else:
        for state_pos in reversed(positions):
            if state_pos.position:
                return (state_pos.position)
    return None    
    
#
"""for each task we know what kind of monitors we should run"""
    
def declare_monitors(longApprehend = True):    
    #pyhop.declare_monitors('pickup_task',clear_block)
#     #unstack_task 
    #pyhop.declare_monitors('unstack_task', clear_block)
#     #unstack
    modified_pyhop.declare_monitors('reach_to_unstack', clear_block)
#     #state.pos[b] == c
#     modified_pyhop.declare_monitors('reach_to_unstack', clear_block)
    modified_pyhop.declare_monitors('reach_to_pickup', clear_block)
   #pickup
    #pyhop.declare_monitors('reach_to_pickup', clear_block)      
#     #get
#     modified_pyhop.declare_monitors('stack', clear_block_stack)
#     