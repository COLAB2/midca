"""
Blocks World methods for Pyhop 1.1.
Author: Dana Nau <nau@cs.umd.edu>, November 15, 2012
This file should work correctly in both Python 2.7 and Python 3.2.
"""


from MIDCA.modules._plan.pyhop import pyhop

class Monitor(object):
    name = {}
    is_active = True
    is_fired = False

    # The class "constructor" - It's actually an initializer 
    def __init__(self, name, isactive, isfired):
        self.name = name
        self.is_active = isactive
        self.is_fired = isfired

    def make_monitor(name):
        monitor = Monitor(name)
        return monitor
    
"""
Here are some helper functions that are used in the methods' preconditions.
"""
#precondition: state.clear[b1] = true
#pickup_task


def pickup_m_monitor(state, b1):
    """generate the monitor 'clear' here for b1"""
    """run the monitor parallel """
    flag = True
    while(flag):
        if state.clear[b1] == False:
            flag = False
    
    pickup_m_monitor_object = filter(lambda x: x.name == pickup_m_monitor, pyhop.monitors)
    pickup_m_monitor_object[0].is_fired = True
    pickup_m_monitor_object[0].is_active = False


def unstack_m_monitor(state,b1):
    """state.clear[b1]"""
    flag = True
    while(flag):
        if state.clear[b1] == False:
            flag = False
    unstack_m_monitor_object = filter(lambda x: x.name == unstack_m_monitor, pyhop.monitors)
    unstack_m_monitor_object[0].is_fired = True
    unstack_m_monitor_object[0].is_active = False
    
def declare_monitors(longApprehend = True):    
    pyhop.declare_monitors('pickup_task', Monitor.make_monitor(pickup_m_monitor))       