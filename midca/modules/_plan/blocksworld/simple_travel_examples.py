"""
The "travel from home to the park" example from my lectures.
Author: Dana Nau <nau@cs.umd.edu>, November 15, 2012
This file should work correctly in both Python 2.7 and Python 3.2.
"""

from __future__ import print_function
import pyhop

import simple_travel_operators
print('')
pyhop.print_operators()

import simple_travel_methods
print('')
pyhop.print_methods()

state1 = pyhop.State('state1')
state1.loc = {'me':'home'}
state1.cash = {'me':20}

# To get multidimensional tables in Python,
# you have to use nested lists
# (or in this case, nested dictionaries)
state1.dist = {'home':{'park':8}}

print("""
****************************************
Call pyhop.pyhop(state1,[('travel','me','home','park')])
with different levels of verbosity
****************************************
""")

print('- verbosity 0:')
pyhop.pyhop(state1,[('travel','me','home','park')])

print('- verbosity 1:')
pyhop.pyhop(state1,[('travel','me','home','park')],verbose=1)

print('- verbosity 2:')
pyhop.pyhop(state1,[('travel','me','home','park')],verbose=2)

print('- verbosity 3:')
pyhop.pyhop(state1,[('travel','me','home','park')],verbose=3)

