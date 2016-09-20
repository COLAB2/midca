"""
Blocks-world test data for Pyhop 1.1.
Author: Dana Nau <nau@cs.umd.edu>, November 15, 2012
This file should work correctly in both Python 2.7 and Python 3.2.
"""

from __future__ import print_function
from MIDCA.modules._plan.pyhop import *



#############     beginning of tests     ################


print("""
****************************************
First, test pyhop on some of the operators and smaller tasks
****************************************
""")

print("- Define state1: a on b, b on tale, c on table")

"""
A state is a collection of all of the state variables and their values. Every state variable in the domain should have a value.
"""

state1 = State('state1')
state1.pos={'a':'table', 'b':'table', 'c':'table'}
state1.clear={'c':True, 'b':True,'a':True}
state1.holding=False

print_state(state1)



print("- Define goal1a:")

"""
A goal is a collection of some (but not necessarily all) of the state variables and their desired values. Below, both goal1a and goal1b specify c on b, and b on a. The difference is that goal1a also specifies that a is on table and the hand is empty.
"""

goal1a = Goal('goal1a')
goal1a.pos={'b':'a'}
goal1a.clear={'c':True, 'b':True, 'a':False}
goal1a.holding=False

print_goal(goal1a)


### goal1b omits some of the conditions of goal1a,
### but those conditions will need to be achieved anyway


pyhop(state1,[('move_blocks', goal1a)], verbose=2)

