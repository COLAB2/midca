from midca import plans, goals
from midca import midcatime
import traceback
import math
import copy
import numpy as np
import zmq, time
import numpy


END_COLOR_CODE = '\033[0m'
NOT_STARTED = 0
NS_COLOR_CODE = END_COLOR_CODE
IN_PROGRESS = 1
IP_COLOR_CODE = '\033[92m'
COMPLETE = 2
C_COLOR_CODE = '\033[94m'
FAILED = 3
F_COLOR_CODE = '\033[91m'

#set this to change output for all asynch actions.
verbose = 2

def asynch_action(mem, midcaAction):
    '''
    returns an asynchronous plan that corresponds to the given MIDCA plan.
    '''
    if midcaAction.op == "commit":
        return Commit(mem, midcaAction)
    if midcaAction.op == "reject":
        return Reject(mem, midcaAction)
    elif midcaAction.op == "inform":
        return Inform(mem, midcaAction)
    elif midcaAction.op == "ask":
        return Request(mem, midcaAction)
    elif midcaAction.op == "wait":
        return wait(mem, midcaAction)
    elif midcaAction.op == "uncommit":
        return uncommit(mem, midcaAction)
    else:
        if verbose >= 1:
            print "MIDCA action", midcaAction, "does not correspond to an asynch",\
                "action. "

    return None

class AsynchAction:

    nextID = 0

    def __init__(self, mem, midcaAction, executeFunc, isComplete, blocks):
        self.status = NOT_STARTED
        self.mem = mem
        self.midcaAction = midcaAction
        self.executeFunc = executeFunc
        self.isComplete = isComplete
        self.blocks = blocks
        self.startTime = None
        self.id = AsynchAction.nextID
        AsynchAction.nextID += 1

    def execute(self):
        if not self.startTime:
            self.startTime = midcatime.now()
        self.status = IN_PROGRESS
        if not self.executeFunc:
            return
        try:
            self.executeFunc(self.mem, self.midcaAction, self.status)
        except:
            if verbose >= 2:
                print "Error executing action", self, ":\n", traceback.format_exc(),
                "\n\nAction assumed to be failed"
            self.status = FAILED

    def check_complete(self):
        if not self.startTime:
            self.startTime = midcatime.now()
        try:
            complete = self.isComplete(self.mem, self.midcaAction, self.status)
            if verbose >= 2 and not complete:
                print "Action", self, "not complete."
            if verbose >= 1 and complete:
                print "Action", self, "complete."
            if complete:
                self.status = COMPLETE
            return complete
        except:
            if verbose >= 1:
                print "Error checking completion status for action", self, " - Assuming \
                 failure:\n", traceback.format_exc()
            self.status = FAILED

    def __str__(self):
        return str(self.midcaAction)

class Commit(AsynchAction):

    '''
    Grace action that senses depth
    '''

    def __init__(self, mem, midcaAction):
        # initialize memory
        self.mem = mem
        self.midcaAction = midcaAction
        self.skip = True
        self.complete = False
        executeAction = lambda mem, midcaAction, status: self.implement_action()
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation()
        AsynchAction.__init__(self, mem, midcaAction, executeAction,
        completionCheck, True)

    def parsable_construct_to_goal(self, goal):
        """
        :param goal: goal
        :return: to a string that can be read by stateread
        """
        # in percieve we replaced the "," with "." to be used as a predicate
        # so to generate a goal we need to reverse it
        # for example Goal[A_. B_. predicate: on] will be Goal(A_, B_, predicate: on)
        goal = str(goal).replace(";",",")
        # convert round braces to square braces because of problem in parsing predicates by stateread
        goal = goal.replace("[","(")
        goal = goal.replace("]",")")

        return goal


    def implement_action(self):
        print ("------------------------------------------------------------------------------------")
        print ("The goal " +self.parsable_construct_to_goal(self.midcaAction.args[2])+ " is committed to achieve ")
        print ("------------------------------------------------------------------------------------")
        pass

    def check_confirmation(self):
        return True
class uncommit(AsynchAction):

    '''
    Grace action that senses depth
    '''

    def __init__(self, mem, midcaAction):
        # initialize memory
        self.mem = mem
        self.midcaAction = midcaAction
        self.skip = True
        self.complete = False
        executeAction = lambda mem, midcaAction, status: self.implement_action()
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation()
        AsynchAction.__init__(self, mem, midcaAction, executeAction,
        completionCheck, True)

    def parsable_construct_to_goal(self, goal):
        """
        :param goal: goal
        :return: to a string that can be read by stateread
        """
        # in percieve we replaced the "," with "." to be used as a predicate
        # so to generate a goal we need to reverse it
        # for example Goal[A_. B_. predicate: on] will be Goal(A_, B_, predicate: on)
        goal = str(goal).replace(";",",")
        # convert round braces to square braces because of problem in parsing predicates by stateread
        goal = goal.replace("[","(")
        goal = goal.replace("]",")")

        return goal


    def implement_action(self):
        print ("------------------------------------------------------------------------------------")
        print ("The goal " +self.parsable_construct_to_goal(self.midcaAction.args[2])+ " is achieved and the agent is no longer committed to the goal")
        print ("------------------------------------------------------------------------------------")
        pass

    def check_confirmation(self):
        return True

class Reject(AsynchAction):

    '''
    Grace action that senses depth
    '''

    def __init__(self, mem, midcaAction):
        # initialize memory
        self.mem = mem
        self.midcaAction = midcaAction
        self.skip = True
        self.complete = False
        executeAction = lambda mem, midcaAction, status: self.implement_action()
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation()
        AsynchAction.__init__(self, mem, midcaAction, executeAction,
        completionCheck, True)

    def parsable_construct_to_goal(self, goal):
        """
        :param goal: goal
        :return: to a string that can be read by stateread
        """
        # in percieve we replaced the "," with "." to be used as a predicate
        # so to generate a goal we need to reverse it
        # for example Goal[A_. B_. predicate: on] will be Goal(A_, B_, predicate: on)
        goal = str(goal).replace(";",",")
        # convert round braces to square braces because of problem in parsing predicates by stateread
        goal = goal.replace("[","(")
        goal = goal.replace("]",")")

        return goal


    def implement_action(self):
        print ("------------------------------------------------------------------------------------")
        print ("The goal " +self.parsable_construct_to_goal(self.midcaAction.args[2])+ " is rejected ")
        print ("------------------------------------------------------------------------------------")
        pass

    def check_confirmation(self):
        return True

class Inform(AsynchAction):

    '''
    Grace action that senses depth
    '''

    def __init__(self, mem, midcaAction):
        # initialize memory
        self.mem = mem
        self.publisher = self.mem.get(self.mem.CONNECTIONS)["publish"]
        self.midcaAction = midcaAction
        self.skip = True
        self.complete = False
        executeAction = lambda mem, midcaAction, status: self.implement_action()
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation()
        AsynchAction.__init__(self, mem, midcaAction, executeAction,
        completionCheck, True)

    def parseGoal(self, txt):
        if not txt.endswith(")"):
            print "Error reading goal. Goal must be given in the form: predicate(arg1, arg2,...,argi-1,argi), where each argument is the name of an object in the world"
            return None
        try:
            if txt.startswith('!'):
                negate = True
                txt = txt[1:]
            else:
                negate = False
            predicateName = txt[:txt.index("(")]
            args = [arg.strip() for arg in txt[txt.index("(") + 1:-1].split(",")]
            #use on-table predicate
            if predicateName == 'on' and len(args) == 2 and 'table' == args[1]:
                predicateName = 'on-table'
                args = args[:1]
            if negate:
                goal = goals.Goal(*args, predicate = predicateName, negate = True)
            else:
                goal = goals.Goal(*args, predicate = predicateName)
            return goal
        except Exception:
            print "Error reading goal. Goal must be given in the form: predicate(arg1, arg2,...,argi-1,argi), where each argument is the name of an object in the world"
            return None

    def parsable_construct_to_goal(self, goal):
        """
        :param goal: goal
        :return: to a string that can be read by stateread
        """
        # in percieve we replaced the "," with "." to be used as a predicate
        # so to generate a goal we need to reverse it
        # for example Goal[A_. B_. predicate: on] will be Goal(A_, B_, predicate: on)
        goal = str(goal).replace(";",",")
        # convert round braces to square braces because of problem in parsing predicates by stateread
        goal = goal.replace("[","(")
        goal = goal.replace("]",")")

        return goal

    def parse_to_atom(self, goal):
        """

        :param goal: convert goal to predicate and arguments
                     on[A;B] to on , A , B
        :return: predicate, arguments
        """
        atom_str =  goal.replace("[", ";").replace("]", "").split(";")
        predicate = atom_str[0]
        arguments = atom_str[1:]
        return predicate, arguments


    def implement_action(self):
        world = self.mem.get(self.mem.STATES)[-1]
        goal = self.parseGoal(self.parsable_construct_to_goal(self.midcaAction.args[2]))
        goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
        if goal in goalGraph:
            if self.midcaAction.args[1] == "human":
                print ("------------------------------------------------------------------------------------")
                print ("The goal " +self.parsable_construct_to_goal(self.midcaAction.args[2])+ " is accepted ")
                print ("------------------------------------------------------------------------------------")
            else:
                self.publisher.send_string("commit: " +  self.parsable_construct_to_goal(self.midcaAction.args[2]))
        else:
            # if the world is achieved
            predicate, args = self.parse_to_atom(self.midcaAction.args[2])
            if world.is_true(predicate, args):
                if self.midcaAction.args[1] == "human":
                    print ("------------------------------------------------------------------------------------")
                    print ("The goal " +self.parsable_construct_to_goal(self.midcaAction.args[2])+ " is achieved ")
                    print ("------------------------------------------------------------------------------------")
                else:
                    self.publisher.send_string("tell: " +  self.parsable_construct_to_goal(self.midcaAction.args[2]))

            else:
                if self.midcaAction.args[1] == "human":
                    print ("------------------------------------------------------------------------------------")
                    print ("The goal " +self.parsable_construct_to_goal(self.midcaAction.args[2])+ " is rejected ")
                    print ("------------------------------------------------------------------------------------")
                else:
                    self.publisher.send_string("reject: " +  self.parsable_construct_to_goal(self.midcaAction.args[2]))
        pass

    def check_confirmation(self):
        return True

class Request(AsynchAction):

    '''
    Grace action that senses depth
    '''

    def __init__(self, mem, midcaAction):
        # initialize memory
        self.mem = mem
        self.publisher = self.mem.get(self.mem.CONNECTIONS)["publish"]
        self.midcaAction = midcaAction
        self.skip = True
        self.complete = False
        executeAction = lambda mem, midcaAction, status: self.implement_action()
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation()
        AsynchAction.__init__(self, mem, midcaAction, executeAction,
        completionCheck, True)

    def parsable_construct_to_goal(self, goal):
        """
        :param goal: goal
        :return: to a string that can be read by stateread
        """
        # in percieve we replaced the "," with "." to be used as a predicate
        # so to generate a goal we need to reverse it
        # for example Goal[A_. B_. predicate: on] will be Goal(A_, B_, predicate: on)
        goal = str(goal).replace(";",",")
        # convert round braces to square braces because of problem in parsing predicates by stateread
        goal = goal.replace("[","(")
        goal = goal.replace("]",")")

        return goal


    def implement_action(self):
        self.publisher.send_string("achieve: " + self.parsable_construct_to_goal(self.midcaAction.args[2]))
        print ("------------------------------------------------------------------------------------")
        print (self.midcaAction.args[0] + " requests the goal " +self.parsable_construct_to_goal(self.midcaAction.args[2])+ " to " +self.midcaAction.args[1] )
        print ("------------------------------------------------------------------------------------")
        pass

    def check_confirmation(self):
        return True

class wait(AsynchAction):

    '''
    Grace action that senses depth
    '''

    def __init__(self, mem, midcaAction):
        # initialize memory
        self.mem = mem
        self.midcaAction = midcaAction
        self.skip = True
        self.complete = False
        executeAction = lambda mem, midcaAction, status: self.implement_action()
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation()
        AsynchAction.__init__(self, mem, midcaAction, executeAction,
        completionCheck, True)

    def parsable_construct_to_goal(self, goal):
        """
        :param goal: goal
        :return: to a string that can be read by stateread
        """
        # in percieve we replaced the "," with "." to be used as a predicate
        # so to generate a goal we need to reverse it
        # for example Goal[A_. B_. predicate: on] will be Goal(A_, B_, predicate: on)
        goal = str(goal).replace(";",",")
        # convert round braces to square braces because of problem in parsing predicates by stateread
        goal = goal.replace("[","(")
        goal = goal.replace("]",")")

        return goal


    def implement_action(self):
        print ("------------------------------------------------------------------------------------")
        print ("The agent is waiting for : " + self.midcaAction.args[1])
        print ("------------------------------------------------------------------------------------")

    def check_confirmation(self):
        world = self.mem.get(self.mem.STATES)[-1]
        atoms = world.get_atoms([self.midcaAction.args[1], self.midcaAction.args[0]])
        for atom in atoms:
            if atom.args[0].name == self.midcaAction.args[1] and atom.args[1].name == self.midcaAction.args[0]\
                    and atom.args[2].name == self.midcaAction.args[2]:
                return True
        return False
