"""
The "travel from home to the park" example from my lectures.
Author: Dana Nau <nau@cs.umd.edu>, November 15, 2012
This file should work correctly in both Python 2.7 and Python 3.2.
"""

import pyhop

def walk(state,a,x,y):
    if state.loc[a] == x:
        state.loc[a] = y
        return state
    else: return False

def call_taxi(state,a,x):
    state.loc['taxi'] = x
    return state
    
def ride_taxi(state,a,x,y):
    if state.loc['taxi']==x and state.loc[a]==x:
        state.loc['taxi'] = y
        state.loc[a] = y
        return state
    else: return False

def pay_driver(state,a,x,y):
    if state.cash[a] >= 1.5 + 0.5 * state.dist[x][y]:
        state.cash[a] = state.cash[a] - (1.5 + 0.5 * state.dist[x][y])
        return state
    else: return False

pyhop.declare_operators(walk, call_taxi, ride_taxi, pay_driver)


