"""
Blocks World methods for Pyhop 1.1.
Author: Dana Nau <nau@cs.umd.edu>, November 15, 2012
This file should work correctly in both Python 2.7 and Python 3.2.
"""


from MIDCA.modules._plan import pyhop
from _io import open

class Monitor():
    
    def __init__(self, name, isactive, isfired, block):
        self.name = name
        self.block = block
        self.is_active = isactive
        self.is_fired = isfired

    
    
"""
Here are some helper functions that are used in the methods' preconditions.
"""
#precondition: state.clear[b1] = true
#pickup_task
#we just generate the monitor for the first task

def clear_block(state, b1):
    #clear_block_object =  filter(lambda x: x.name == clear_block, pyhop.monitors)
#     if blocks.blocks[b1].type.name != "BLOCK":
#         return True
    #f = open('myfile','w')
     # python will convert \n to os.linesep
   
    
    for key in pyhop.monitors.keys():
        m_list = pyhop.monitors[key]
        for m in m_list:
            if m.name == clear_block: 
                print("@@" + m.name.__name__ + " " + str(m.is_active))
                
                if m.block == b1 and m.is_active == True:
                    print "it is already running"
                else:
                    m.is_active = True
                    m.block = b1
                    print("monitor is running for " + b1)
                    
                    while(m.is_fired == False):
                        #f.write (b1 + " true")
                        
                        if state.clear[b1] == False:
                            m.is_fired = True
                break

"""for each task we know what kind of monitors we should run"""
    
def declare_monitors(longApprehend = True):    
    pyhop.declare_monitors('pickup_task', Monitor(clear_block, False, False, None))       