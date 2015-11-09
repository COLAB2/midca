"""
Blocks World methods for Pyhop 1.1.
Author: Dana Nau <nau@cs.umd.edu>, November 15, 2012
This file should work correctly in both Python 2.7 and Python 3.2.
"""


from MIDCA.modules._plan import pyhop
from _io import open
import time

class Monitor():
    
    def __init__(self, name, block, depth, task_name):
        self.name = name
        self.block = block
        self.depth = depth
        self.is_active = True
        self.is_fired = False
        self.task_name = task_name

    
    
"""
Here are some helper functions that are used in the methods' preconditions.
"""
#precondition: state.clear[b1] = true
#pickup_task
#we just generate the monitor for the first task




def clear_block(state, depth, b1, task_name):
    
    i = 0
    m =  filter(lambda x: x.name.__name__ == clear_block and x.block == b1, pyhop.generated_monitors)
    if m:     
        print "monitor is already running for " + b1
    else:
        m = Monitor(clear_block, b1, depth, task_name)
        pyhop.generated_monitors.append(m)
    
        while(m.is_fired == False):
                        #f.write (b1 + " true")
            i = i + 1
            time.sleep(3)
            if i > 7:
                m.is_fired = True
                print("monitor: " + b1 + "is not clear!")
                        
            if state.clear[b1] == False:
                print("monitor: " + b1 + "is not clear!")
                m.is_fired = True
               

# def clear_block(state, depth, b1):
#     #clear_block_object =  filter(lambda x: x.name == clear_block, pyhop.monitors)
# #     if blocks.blocks[b1].type.name != "BLOCK":
# #         return True
#     #f = open('myfile','w')
#      # python will convert \n to os.linesep
#    
#     #we don'r want to generate a monitor for a block again
#     #
#     for key in pyhop.monitors.keys():
#         m_list = pyhop.monitors[key]
#         for m in m_list:
#             if m.name == clear_block and m.block == b1 and m.is_active == True: 
#                 print "monitor is already running for " + b1
#                 break
#             
#             elif m.name == clear_block and m.block != b1:   
#                 i = 0
#                 m.is_active = True
#                 m.block = b1
#                 m.depth = depth
#                 print "monitor is running for pickup_task block " + b1 
#                     
#                 while(m.is_fired == False):
#                         #f.write (b1 + " true")
#                     i = i + 1
#                     time.sleep(3)
#                     if i > 7:
#                         m.is_fired = True
#                         print("monitor: " + b1 + "is not clear!")
#                         
#                     if state.clear[b1] == False:
#                         print("monitor: " + b1 + "is not clear!")
#                         m.is_fired = True
#                 break        
#                

"""for each task we know what kind of monitors we should run"""
    
def declare_monitors(longApprehend = True):    
    pyhop.declare_monitors('pickup_task', Monitor(clear_block, False, False, "None"))       