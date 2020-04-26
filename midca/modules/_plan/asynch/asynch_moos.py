from midca import rosrun, plans
from midca import midcatime
import traceback
import math
import copy
import numpy as np
import zmq, time
import numpy

try:
    # moos specific import to perform apprehend and report actions
    from midca.domains.moos_domain import moosworld
except:
    print ("Moos specific imports failed")


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


def get_asynch_action(midcaAction):
    raise ArgumentException("midca action " + str(midcaAction)  + " does not translate to a \
    valid asynchronous action.")

def asynch_plan(mem, midcaPlan):
    '''
    returns an asynchronous plan that corresponds to the given MIDCA plan.
    '''
    actions = []
    goals = midcaPlan.goals
    for midcaAction in midcaPlan.actions:
        if midcaAction.op == "ignore":
            actions.append(IgnoreMine(mem, midcaAction))

        elif midcaAction.op == "remove":
            actions.append(RemoveMine(mem, midcaAction))

        elif midcaAction.op == "remove_mines":
            actions.append(RemoveMines(mem, midcaAction))

        elif midcaAction.op == "fast_survey":
            actions.append(FastSurvey(mem, midcaAction))

        elif midcaAction.op == "slow_survey":
            actions.append(SlowSurvey(mem, midcaAction))

        elif midcaAction.op == "apprehend":
            actions.append(Apprehend(mem, midcaAction))

        elif midcaAction.op == "report":
            actions.append(Report(mem, midcaAction))

        elif midcaAction.op == "reach_to_catch":
            actions.append(CatchEnemy(mem, midcaAction))

        else:
            if verbose >= 1:
                print "MIDCA action", midcaAction, "does not correspond to an asynch",
                "action. MIDCA will skip this action"

    return AsynchPlan(actions, goals)

class AsynchPlan(plans.Plan):

    '''
    subclass of MIDCA Plan class that uses asynchronous actions.
    '''

    def finished(self):
        '''
        overrides plan.finished(). Declares a plan complete if all its actions report
        complete or failed.
        '''
        for action in self.actions:
            if action.status != COMPLETE and action.status != FAILED:
                return False
        return True

    @property
    def status(self):
        '''
        property that returns the plan's status. This can be NOT_STARTED, IN_PROGRESS,
        FAILED, or COMPLETE. If any action fails, the plan is considered to have failed.
        The plan is complete when all actions are complete.
        '''
        status = COMPLETE
        for action in self.actions:
            if action.status == FAILED:
                return FAILED
            elif action.status == NOT_STARTED and status == COMPLETE:
                status = NOT_STARTED
            elif action.status == IN_PROGRESS:
                status = IN_PROGRESS
        return status

    def __str__(self):
        s = ""
        for action in self.actions:
            if action.status == NOT_STARTED:
                s += NS_COLOR_CODE
            elif action.status == IN_PROGRESS:
                s += IP_COLOR_CODE
            elif action.status == FAILED:
                s += F_COLOR_CODE
            elif action.status == COMPLETE:
                s += C_COLOR_CODE

            s += str(action) + " "
        return s[:-1] + END_COLOR_CODE

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

    def ros_msg(self, topic, d):
        '''
        arg d should be a dictionary that contains the key/value pairs to be sent.
        '''
        sent = rosrun.send_msg(topic, rosrun.dict_as_msg)
        if not sent:
            if verbose >= 1:
                print "Unable to send msg; ", d, "on topic", topic, " Action", self,
                "assumed failed."
            self.status = FAILED

    def __str__(self):
        return str(self.midcaAction)

class IgnoreMine(AsynchAction):

    '''
    Grace action that senses depth
    '''

    def __init__(self, mem, midcaAction):
        # zeromq connection
        context = zmq.Context()
        self.publisher = context.socket(zmq.PUB)
        self.publisher.connect("tcp://127.0.0.1:5560")

        # initialize memory
        self.mem = mem
        self.midcaAction = midcaAction
        self.skip = True
        self.complete = False
        executeAction = lambda mem, midcaAction, status: self.implement_action()
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation()
        AsynchAction.__init__(self, mem, midcaAction, executeAction,
        completionCheck, True)

    def implement_action(self):
        time.sleep(0.25)
        for i in range(2):
            self.publisher.send_multipart([b"M", b"speed =0.0"])


    def check_confirmation(self):
        if self.skip:
            self.skip = False
            return False
        self.mem.set(self.mem.PAUSE, True)
        return True

class RemoveMines(AsynchAction):

    '''
    Grace action that senses depth
    '''

    def __init__(self, mem, midcaAction):

        # initialize memory
        self.mem = mem
        self.midcaAction = midcaAction
        self.complete = False
        executeAction = lambda mem, midcaAction, status: self.implement_action()
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation()
        AsynchAction.__init__(self, mem, midcaAction, executeAction,
        completionCheck, True)

    def implement_action(self):
        pass


    def check_confirmation(self):
       return True

class RemoveMine(AsynchAction):

    '''
    Grace action that senses depth
    '''

    def __init__(self, mem, midcaAction):
        # zeromq connection
        context = zmq.Context()
        self.publisher = context.socket(zmq.PUB)
        self.publisher_mine = context.socket(zmq.PUB)
        self.publisher.connect("tcp://127.0.0.1:5560")
        self.publisher_mine.connect("tcp://127.0.0.1:5565")

        # variable to skip 1 cycle to execute this action
        self.skip = True

        # initialize memory
        self.mem = mem
        self.midcaAction = midcaAction
        self.complete = False
        executeAction = lambda mem, midcaAction, status: self.implement_action(midcaAction)
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation()
        AsynchAction.__init__(self, mem, midcaAction, executeAction,
        completionCheck, True)

    def implement_action(self, action):
        time.sleep(0.3)
        label= int(action.args[0].replace("mine",""))
        self.publisher_mine.send_multipart([b"M", str(label)])
        time.sleep(0.3)
        for i in range(2):
            self.publisher.send_multipart([b"M", b"speed =0.0"])
            time.sleep(0.1)
        # assign scores
        if (action.args[1] == "ga1" or action.args[1] == "ga2"):
            score = self.mem.get(self.mem.MOOS_SCORE) + 1
            self.mem.set(self.mem.MOOS_SCORE, score)

        elif (action.args[1] == "qroute"):
            score = self.mem.get(self.mem.MOOS_SCORE) + 1
            self.mem.set(self.mem.MOOS_SCORE, score)
        else:
            pass

    def check_confirmation(self):
        if self.skip:
            self.skip = False
            return False
        time.sleep(0.1)
        self.mem.set(self.mem.PAUSE, True)
        #self.publisher.send_multipart([b"M", b"speed = 0.5"])
        return True

class FastSurvey(AsynchAction):

    '''
    Grace action that senses depth
    '''

    def __init__(self, mem, midcaAction):
        # zeromq connection
        context = zmq.Context()
        self.publisher = context.socket(zmq.PUB)
        self.publisher.connect("tcp://127.0.0.1:5560")

        # initialize memory
        self.mem = mem
        self.midcaAction = midcaAction
        self.complete = False
        executeAction = lambda mem, midcaAction, status: self.implement_action(midcaAction)
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation(midcaAction)
        AsynchAction.__init__(self, mem, midcaAction, executeAction,
        completionCheck, True)

    def implement_action(self, action):

        time.sleep(0.25)

        argnames = [str(arg) for arg in action.args]

        if ("transit1" in argnames):
            message = [b"M", b"point = -25,-43 # speed= 1.0"]
            self.publisher.send_multipart(message)

        elif ("qroute_transit" in argnames):
            message = [b"M", b"point = 41,-119 # speed= 1.0"]
            for i in range(2):
                self.publisher.send_multipart(message)

        elif ("home" in argnames):
            message = [b"M", b"point = 238,83 # speed= 1.0"]
            for i in range(2):
                    self.publisher.send_multipart(message)

        elif ("transit2" in argnames):
            message = [b"M", b"point = 135,-44 # speed= 1.0"]
            for i in range(2):
                self.publisher.send_multipart(message)

        elif ("ga3" in argnames):
            message = [b"M",b" points=format=lawnmower,label=dedley_survey, x=149, y=-194, width=30, height = 30,lane_width=10, rows=north-south,degs=0 # speed =1.0"]
            for i in range(2):
                self.publisher.send_multipart(message)


    def check_confirmation(self, action):
        world = self.mem.get(self.mem.STATES)[-1]
        argnames = [str(arg) for arg in action.args]
        for atom in world.atoms:
          if atom.predicate.name == "at_location" and \
                  atom.args[0].name == argnames[0] and \
                  atom.args[1].name == argnames[1]:
            return True

        pause = self.mem.get(self.mem.PAUSE)
        if pause:
            for i in range(2):
                self.publisher.send_multipart([b"M", b"speed = 1.0"])
                time.sleep(0.1)
                self.mem.set(self.mem.PAUSE, False)
            for i in range(2):
                self.publisher.send_multipart([b"M", b"speed = 1.0"])
                time.sleep(0.1)

        else:
            speed = self.mem.get(self.mem.REMUS_SPEED)
            if speed == 0:
                self.implement_action(action)


        if argnames[1] == "ga3":
            atomic_world = world.copy()
            flag = 0
            for atom in atomic_world.atoms:
                if atom.predicate.name == "hazard_at_location":
                    world.remove_atom(atom)
                    flag = 1
            if flag:
                self.mem.add(self.mem.STATES, world)
            goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
            goals =  goalGraph.getAllGoals()
            for each_goal in goals:
                if each_goal["predicate"] == "hazard_checked":
                    goalGraph.remove(each_goal)
                    goalGraph.removeOldPlans()

        return False

class SlowSurvey(AsynchAction):

    '''
    Grace action that senses depth
    '''

    def __init__(self, mem, midcaAction):
        # zeromq connection
        context = zmq.Context()
        self.publisher = context.socket(zmq.PUB)
        self.publisher.connect("tcp://127.0.0.1:5560")

        # initialize memory
        self.mem = mem
        self.midcaAction = midcaAction
        self.complete = False
        self.wait = 0
        executeAction = lambda mem, midcaAction, status: self.implement_action(midcaAction)
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation(midcaAction)
        AsynchAction.__init__(self, mem, midcaAction, executeAction,
        completionCheck, True)

    def implement_action(self, action):

        time.sleep(0.25)

        argnames = [str(arg) for arg in action.args]

        if ("ga1" in argnames):
            message = [b"M",b"   points=format=lawnmower,label=dedley_survey, x=-22, y=-128, width=50, height = 50,lane_width=10, rows=north-south,degs=0  # speed =0.5"]
            print ("Start Surveying ....")
            for i in range(2):
                self.publisher.send_multipart(message)

        if ("ga2" in argnames):
            message = [b"M",b" points=format=lawnmower,label=dedley_survey, x=139, y=-128, width=50, height = 50,lane_width=10, rows=north-south,degs=0  # speed =0.5"]
            for i in range(2):
                self.publisher.send_multipart(message)

        if ("home" in argnames):
            message = [b"M", b"point = 238,83 # speed= 0.5"]
            for i in range(2):
                    self.publisher.send_multipart(message)

        if ("way_point" in argnames):
            # get the go to way_points from memory
            way_points = self.mem.get(self.mem.WAY_POINTS)
            # compute the string using way_points
            # fromat is "x,y:x1,y1:x2,y2:...."
            # sample string "12,35:15,38"
            message = way_points["message"]
            for i in range(2):
                self.publisher.send_multipart(message)
                time.sleep(0.1)



    def check_confirmation(self, action):
        world = self.mem.get(self.mem.STATES)[-1]
        argnames = [str(arg) for arg in action.args]

        for atom in world.atoms:
          if atom.predicate.name == "at_location" and \
                  atom.args[0].name == argnames[0] and \
                  atom.args[1].name == argnames[1]:

            if ("way_point" in argnames):
                self.mem.set(self.mem.explanations, None)
            return True

        if self.mem.get(self.mem.apprehended):
            speed = self.mem.get(self.mem.REMUS_SPEED)
            if speed == 0:
                self.implement_action(action)
                time.sleep(2)
            else:
                if self.wait == 4:
                    self.mem.set(self.mem.apprehended, False)
                time.sleep(2)
                self.wait+=1
                return False

        pause = self.mem.get(self.mem.PAUSE)
        if pause:
            self.mem.set(self.mem.PAUSE, False)
            self.publisher.send_multipart([b"M", b"speed = 0.5"])
            time.sleep(0.1)
            self.publisher.send_multipart([b"M", b"speed = 0.5"])
            for i in range(2):
                self.publisher.send_multipart([b"M", b"speed = 0.5"])
                time.sleep(0.1)

        else:
            speed = self.mem.get(self.mem.REMUS_SPEED)
            if speed == 0:
                self.implement_action(action)
        return False

class CatchEnemy(AsynchAction):

    '''
    Grace action that senses depth
    '''

    def __init__(self, mem, midcaAction):
        # zeromq connection
        context = zmq.Context()
        self.publisher = context.socket(zmq.PUB)
        self.publisher.connect("tcp://127.0.0.1:5560")


        # initialize memory
        self.mem = mem
        self.midcaAction = midcaAction
        self.complete = False
        executeAction = lambda mem, midcaAction, status: self.implement_action(midcaAction)
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation(midcaAction)
        AsynchAction.__init__(self, mem, midcaAction, executeAction,
        completionCheck, True)

    def implement_action(self, action):
        time.sleep(0.25)
        argnames = [str(arg) for arg in action.args]
        enemy_location = self.mem.get(self.mem.ENEMY_LOCATION)[argnames[1]]
        message = [b"M", b"point = "+str(enemy_location[0]) + "," +str(enemy_location[1])+ "# speed= 0.8"]
        self.publisher.send_multipart(message)

    def check_confirmation(self, action):
        argnames = [str(arg) for arg in action.args]
        enemy_location = self.mem.get(self.mem.ENEMY_LOCATION)[argnames[1]]
        agent_location = self.mem.get(self.mem.AGENT_LOCATION)
        distance = math.sqrt(sum([(a - b) ** 2 for a, b in zip(enemy_location, agent_location)]))
        if distance >= 30:
                message = [b"M", b"point = "+str(enemy_location[0]) + "," +str(enemy_location[1])+ "# speed= 0.8"]
                self.publisher.send_multipart(message)
                return False
        else:
            return True

class Apprehend(AsynchAction):

    '''
    Grace action that senses depth
    '''

    def __init__(self, mem, midcaAction):
        # initialize memory
        self.mem = mem
        self.midcaAction = midcaAction
        context = zmq.Context()
        self.publisher = context.socket(zmq.PUB)
        self.publisher.connect("tcp://127.0.0.1:5560")

        self.fisher1 = context.socket(zmq.PUB)
        self.fisher1.connect("tcp://127.0.0.1:5595")

        self.fisher2 = context.socket(zmq.PUB)
        self.fisher2.connect("tcp://127.0.0.1:5598")

        self.fisher3 = context.socket(zmq.PUB)
        self.fisher3.connect("tcp://127.0.0.1:5601")

        self.vessels = {"fisher1": self.fisher1,
                        "fisher2": self.fisher2,
                        "fisher3": self.fisher3,
                        "fisher4": self.publisher}

        self.skip = True
        self.complete = False
        executeAction = lambda mem, midcaAction, status: self.implement_action(midcaAction)
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation(midcaAction)
        AsynchAction.__init__(self, mem, midcaAction, executeAction,
        completionCheck, True)

    def implement_action(self, action):
        argnames = [str(arg) for arg in action.args]
        if argnames[1] == "fisher4":
            moosworld.pirate_flag = True
        else:
            self.vessels[argnames[1]].send_multipart([b"M", b"speed = 0.0"])

        self.publisher.send_multipart([b"M", b"speed = 0.0"])

    def check_confirmation(self, action):
        world = self.mem.get(self.mem.STATES)[-1]
        if self.skip:
            self.skip = False
            return False

        self.mem.set(self.mem.PAUSE, False)

        atomic_world = world.copy()
        flag = 0
        for atom in atomic_world.atoms:
            if atom.predicate.name == "hazard_at_location":
                world.remove_atom(atom)
                flag = 1
        if flag:
            self.mem.add(self.mem.STATES, world)

        goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
        goals =  goalGraph.getAllGoals()
        for each_goal in goals:
            if each_goal["predicate"] == "hazard_checked":
                goalGraph.remove(each_goal)
                goalGraph.removeOldPlans()


        self.mem.set(self.mem.apprehended, True)
        return True

class Report(AsynchAction):

    '''
    Grace action that senses depth
    '''

    def __init__(self, mem, midcaAction):
        # initialize memory
        self.mem = mem
        self.midcaAction = midcaAction
        self.skip = True
        self.complete = False
        executeAction = lambda mem, midcaAction, status: self.implement_action(midcaAction)
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation(midcaAction)
        AsynchAction.__init__(self, mem, midcaAction, executeAction,
        completionCheck, True)

    def implement_action(self, action):
        argnames = [str(arg) for arg in action.args]
        # get the index from the argument 1 (ex ship1)
        vessel_index = int(argnames[1].replace("ship", ""))
        moosworld.change_qroute[vessel_index] =True


    def check_confirmation(self, action):
        world = self.mem.get(self.mem.STATES)[-1]
        if self.skip:
            self.skip = False
            return False

        atomic_world = world.copy()
        flag = 0
        for atom in atomic_world.atoms:
            if atom.predicate.name == "hazard_at_location":
                world.remove_atom(atom)
                flag = 1
        if flag:
            self.mem.add(self.mem.STATES, world)

        goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
        goals =  goalGraph.getAllGoals()
        for each_goal in goals:
            if each_goal["predicate"] == "hazard_checked":
                goalGraph.remove(each_goal)
                goalGraph.removeOldPlans()
        return True
