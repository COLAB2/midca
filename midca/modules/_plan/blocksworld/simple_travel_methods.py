"""
The "travel from home to the park" example from my lectures.
Author: Dana Nau <nau@cs.umd.edu>, November 15, 2012
This file should work correctly in both Python 2.7 and Python 3.2.
"""

import pyhop

def travel_by_foot(state,a,x,y):
    if state.dist[x][y] <= 2:
        return [('walk',a,x,y)]
    return False

def travel_by_taxi(state,a,x,y):
    if state.cash[a] >= 1.5 + 0.5 * state.dist[x][y]:
        return [('call_taxi',a,x), ('ride_taxi',a,x,y), ('pay_driver',a,x,y)]
    return False


pyhop.declare_methods('travel',travel_by_foot,travel_by_taxi)
