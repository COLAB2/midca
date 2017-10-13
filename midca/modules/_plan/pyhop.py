"""
Pyhop, version 1.2.1 -- a simple SHOP-like planner written in Python.
Author: Dana S. Nau, 15 February 2013

Copyright 2013 Dana S. Nau - http://www.cs.umd.edu/~nau

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
   
Pyhop should work correctly in both Python 2.7 and Python 3.2.
For examples of how to use it, see the example files that come with Pyhop.

Pyhop provides the following classes and functions:

- foo = State('foo') tells Pyhop to create an empty state object named 'foo'.
  To put variables and values into it, you should do assignments such as
  foo.var1 = val1

- bar = Goal('bar') tells Pyhop to create an empty goal object named 'bar'.
  To put variables and values into it, you should do assignments such as
  bar.var1 = val1

- print_state(foo) will print the variables and values in the state foo.

- print_goal(foo) will print the variables and values in the goal foo.

- declare_operators(o1, o2, ..., ok) tells Pyhop that o1, o2, ..., ok
  are all of the planning operators; this supersedes any previous call
  to declare_operators.

- print_operators() will print out the list of available operators.

- declare_methods('foo', m1, m2, ..., mk) tells Pyhop that m1, m2, ..., mk
  are all of the methods for tasks having 'foo' as their taskname; this
  supersedes any previous call to declare_methods('foo', ...).

- print_methods() will print out a list of all declared methods.

- pyhop(state1,tasklist) tells Pyhop to find a plan for accomplishing tasklist
  (a list of tasks), starting from an initial state state1, using whatever
  methods and operators you declared previously.

- In the above call to pyhop, you can add an optional 3rd argument called
  'verbose' that tells pyhop how much debugging printout it should provide:
- if verbose = 0, then pyhop prints nothing;
- if verbose = 1, it prints the initial parameters and the answer;
- if verbose = 2, it also prints a message on each recursive call;
- if verbose = 3, it also prints info about what it's computing.
"""

# Like the SHOP and JSHOP planners (see http://www.cs.umd.edu/projects/shop),
# Pyhop uses HTN methods to decompose tasks into smaller and smaller
# subtasks, until it finds tasks that correspond directly to actions.
# But several of the details are different:
# 
# (1) In SHOP and JSHOP, one uses a special-purpose language to write HTN
#     methods and planning operators -- but in Pyhop, one writes the methods
#     and operators as ordinary Python functions. This should make it easier
#     to use Pyhop as part of other programs. 
# 
# (2) Pyhop represents states as collections of variables, not collections
#     of logical assertions. For example, to say that box b is in room r1,
#     instead of writing an assertion such as "in(b,r1)", you would write
#     something like "loc[b] = r1".
# 
# (3) The current state is a Python object. The state variables are part of
#     that object, and you need to refer to this object explicitly in the
#     operator and method definitions. Thus, what you'd *really* write in
#     the above example is something like this:
#     s = State()
#     s.loc['b'] = 'r1'
# 
# (4) You also can define a goal as a Python object. For example, to specify
#     that your goal is to have box b in room r2, you might write this:
#     g = Goal()
#     g.loc['b'] = 'r2'
#     Pyhop doesn't explicitly check to see if the goal is achieved. But you
#     can pass the goal object as an argument to your operators and methods,
#     so that their preconditions and effects can refer to it. If you want to
#     accomplish a sequence of goals, one at a time (e.g., first achieve g1,
#     then g2, then g3), you could define all three of them as goal objects,
#     pass all three of them as arguments to your operators and methods.
# 
# (5) Unlike SHOP and JSHOP, Pyhop doesn't include a Horn-clause inference
#     engine for use in evaluating preconditions. So far, I haven't seen any
#     need for it; I've found it easier to write precondition-evaluation
#     functions directly in Python. But I'll consider adding Horn-clause
#     inference to Pyhop if someone convinces me that it's really necessary.
# 
# Accompanying this file is a file called examples.py that provides examples
# of how to use Pyhop. To run it, launch python and type 'import examples'.


from __future__ import print_function
import copy,sys, pprint

############################################################
# States and goals

class State():
    """A state is just a collection of variable bindings."""
    def __init__(self,name):
        self.__name__ = name

class Goal():
    """A goal is just a collection of variable bindings."""
    def __init__(self,name):
        self.__name__ = name        


### print_state and print_goal are identical except for the name

def print_state(state,indent=4):
    """Print each variable in state, indented by indent spaces."""
    if state != False:
        for (name,val) in vars(state).items():
            if name != '__name__':
                for x in range(indent): sys.stdout.write(' ')
                sys.stdout.write(state.__name__ + '.' + name)
                print(' =', val)
    else: print('False')

def print_goal(goal,indent=4):
    """Print each variable in goal, indented by indent spaces."""
    if goal != False:
        for (name,val) in vars(goal).items():
            if name != '__name__':
                for x in range(indent): sys.stdout.write(' ')
                sys.stdout.write(goal.__name__ + '.' + name)
                print(' =', val)
    else: print('False')

############################################################
# Helper functions that may be useful in domain models

def forall(seq,cond):
    """True if cond(x) holds for all x in seq, otherwise False."""
    for x in seq:
        if not cond(x): return False
    return True

def find_if(cond,seq):
    """
    Return the first x in seq such that cond(x) holds, if there is one.
    Otherwise return None.
    """
    for x in seq:
        if cond(x): return x
    return None

############################################################
# Commands to tell Pyhop what the operators and methods are

operators = {}
methods = {}

def declare_operators(*op_list):
    """
    Call this after defining the operators, to tell Pyhop what they are. 
    op_list must be a list of functions, not strings.
    """
    operators.update({op.__name__:op for op in op_list})
    return operators

def declare_methods(task_name,*method_list):
    """
    Call this once for each task, to tell Pyhop what the methods are.
    task_name must be a string.
    method_list must be a list of functions, not strings.
    """
    methods.update({task_name:list(method_list)})
    return methods[task_name]

############################################################
# Commands to find out what the operators and methods are

def print_operators(olist=operators):
    """Print out the names of the operators"""
    print('OPERATORS:', ', '.join(olist))

def print_methods(mlist=methods):
    """Print out a table of what the methods are for each task"""
    print('{:<14}{}'.format('TASK:','METHODS:'))
    for task in mlist:
        print('{:<14}'.format(task) + ', '.join([f.__name__ for f in mlist[task]]))

############################################################
# The actual planner

def pyhop(state,tasks,verbose=0):
    """
    Try to find a plan that accomplishes tasks in state. 
    If successful, return the plan. Otherwise return False.
    """
    if verbose>0: print('** pyhop:\n   state = {}\n   tasks = {}'.format(state.__name__,tasks))
    result = seek_plan(state,tasks,[],0,verbose)
    if verbose>0: print('** result =',result,'\n')
    return result

def copy_state(state):
    try:
        return state.copy()
    except AttributeError:
        return copy.deepcopy(state)

def seek_plan(state,tasks,plan,depth,verbose=0):
    """
    Workhorse for pyhop. state and tasks are as in pyhop.
    - plan is the current partial plan.
    - depth is the recursion depth, for use in debugging
    - verbose is whether to print debugging messages
    """
    if verbose>1: print('depth {} tasks {}'.format(depth,tasks))
    if tasks == []:
        if verbose>2: print('depth {} returns plan {}'.format(depth,plan))
        return plan
    task1 = tasks[0]
    if task1[0] in operators:
        if verbose>2: print('depth {} action {}'.format(depth,task1))
        operator = operators[task1[0]]
        newstate = operator(copy_state(state),*task1[1:])
        if verbose>2:
            print('depth {} new state:'.format(depth))
            print_state(newstate)
        if newstate:
            solution = seek_plan(newstate,tasks[1:],plan+[task1],depth+1,verbose)
            if solution != False:
                return solution
    if task1[0] in methods:
        if verbose>2: print('depth {} method instance {}'.format(depth,task1))
        relevant = methods[task1[0]]
        for method in relevant:
            subtasks = method(state,*task1[1:])
            # Can't just say "if subtasks:", because that's wrong if subtasks == []
            if verbose>2:
                print('depth {} new tasks: {}'.format(depth,subtasks))
            if subtasks != False:
                solution = seek_plan(state,subtasks+tasks[1:],plan,depth+1,verbose)
                if solution != False:
                    return solution
    if verbose>2: print('depth {} returns failure'.format(depth))
    return False
