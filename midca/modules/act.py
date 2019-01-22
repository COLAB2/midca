from midca.modules._plan.asynch import asynch
from midca import base
import copy
import zmq, math
import time


class AsynchronousAct(base.BaseModule):

    '''
    MIDCA module that "executes" plans in which the individual actions will be conducted
    asynchronously. This was originally designed to allow MIDCA to work as a robot
    controller in communication with ROS sensor and effector nodes.
    '''

    def run(self, cycle, verbose = 2):
        self.verbose = verbose
        try:
            goals = self.mem.get(self.mem.CURRENT_GOALS)[-1]
        except:
            goals = []
        if not goals:
            if verbose >= 2:
                print "No Active goals. Act phase will do nothing"
            return

        try:
            plan = self.mem.get(self.mem.GOAL_GRAPH).getMatchingPlan(goals)
        except:
            if verbose >= 1:
                print "Error loading plan. Skipping act phase."
            return

        if not plan:
            if verbose > 2:
                print "No current plan. Skipping Act phase"
            return
        i = 0
        if plan.finished():
            print "Plan", plan, "has already been completed"
            return
        #ideally MIDCA should check for other valid plans, but for now it doesn't.

        while i < len(plan):
            action = plan[i]
            try:
                if action.status != asynch.FAILED and action.status != asynch.COMPLETE:
                    completed = action.check_complete()
                    if completed:
                        if verbose >= 2:
                            print "Action", action, "completed"
            except AttributeError:
                if verbose >= 1:
                    print "Action", action, "Does not seem to have a valid check_complete() ",
                    "method. Therefore MIDCA cannot execute it."
                    action.status = asynch.FAILED
            try:
                if action.status == asynch.NOT_STARTED:
                    if verbose >= 2:
                        print "Beginning action execution for", action
                    action.execute()
            except AttributeError:
                if verbose >= 1:
                    print "Action", action, "Does not seem to have a valid execute() ",
                    "method. Therefore MIDCA cannot execute it"
                    action.status = asynch.FAILED
            if action.status == asynch.COMPLETE:
                i += 1
            elif not action.blocks:
                i += 1
            else:
                break

class SimpleAct(base.BaseModule):

    '''
    MIDCA module that selects the plan, if any, that achieves the most current goals, then selects the next action from that plan. The selected action is stored in a two-dimensional array in mem[mem.ACTIONS], where mem[mem.ACTIONS][x][y] returns the yth action to be taken at time step x. So mem[mem.ACTIONS][-1][0] is the last action selected. Note that this will throw an index error if no action was selected.
    To have MIDCA perform multiple actions in one cycle, simple add several actions to mem[mem.ACTIONS][-1]. So mem[mem.ACTIONS][-1][0] is the first action taken, mem[mem.ACTIONS][-1][1] is the second, etc.
    '''

    #returns the plan that achieves the most current goals, based on simulation.
    def get_best_plan(self, world, goals, verbose):
        plan = None
        goalsAchieved = set()
        goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
        for nextPlan in goalGraph.allMatchingPlans(goals):
            achieved = world.goals_achieved(nextPlan, goals)
            if len(achieved) > len(goalsAchieved):
                goalsAchieved = achieved
                plan = nextPlan
            if len(achieved) == len(goals):
                break
            elif verbose >= 2:
                print "Retrieved plan does not achieve all goals. Trying to retrieve a different plan..."
                if verbose >= 3:
                    print "  Retrieved Plan:"
                    for a in nextPlan:
                        print "  "+str(a)
                    print "Goals achieved:", [str(goal) for goal in achieved]
        if plan == None and verbose >= 1:
            print "No valid plan found that achieves any current goals."
        elif len(goalsAchieved) < len(goals) and verbose >= 1:
            print "Best plan does not achieve all goals."
            if verbose >= 2:
                print "Plan:", str(plan)
                print "Goals achieved:", [str(goal) for goal in goalsAchieved]
        return plan

    def run(self, cycle, verbose = 2):
        self.verbose = verbose
        max_plan_print_size = 5
        world = self.mem.get(self.mem.STATES)[-1]
        try:
            goals = self.mem.get(self.mem.CURRENT_GOALS)[-1]
        except :
            goals = []
        plan = self.get_best_plan(world, goals, verbose)
        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("WORLD", copy.deepcopy(world))
            trace.add_data("GOALS", copy.deepcopy(goals))
            trace.add_data("PLAN", copy.deepcopy(plan))

        if plan != None:
            action = plan.get_next_step()
            if not action:
                if verbose >= 1:
                    print "Plan to achieve goals has already been completed. Taking no action."
                self.mem.add(self.mem.ACTIONS, [])
            else:
                if verbose == 1:
                    print "Action selected:", action
                elif verbose >= 2:
                    if len(plan) > max_plan_print_size:
                        # print just the next 3 actions of the plan
                        print "Selected action", action, "from plan:\n"
                        if verbose >= 3:
                            for a in plan:
                                print "  "+str(a)
                    else:
                        # print the whole plan
                        print "Selected action", action, "from plan:\n", plan
                self.mem.add(self.mem.ACTIONS, [action])
                actions = self.mem.get(self.mem.ACTIONS)
                if len(actions) > 400:
                    actions = actions[200:] # trim off old stale actions
                    self.mem.set(self.mem.ACTIONS, actions)
                    #print "Trimmed off 200 old stale actions to save space"
                plan.advance()

                if trace: trace.add_data("ACTION", action)
        else:
            if verbose >= 1:
                print "MIDCA will not select an action this cycle."
            self.mem.add(self.mem.ACTIONS, [])
            if goals:
                for g in goals:
                    self.mem.get(self.mem.GOAL_GRAPH).remove(g)

            if trace: trace.add_data("ACTION", None)


class SimpleAct_temporary(base.BaseModule):
    '''
    For both construction and restaurant domain
    MIDCA module that selects the plan, if any, that achieves the most current goals, then selects the next action from that plan. The selected action is stored in a two-dimensional array in mem[mem.ACTIONS], where mem[mem.ACTIONS][x][y] returns the yth action to be taken at time step x. So mem[mem.ACTIONS][-1][0] is the last action selected. Note that this will throw an index error if no action was selected.
    To have MIDCA perform multiple actions in one cycle, simple add several actions to mem[mem.ACTIONS][-1]. So mem[mem.ACTIONS][-1][0] is the first action taken, mem[mem.ACTIONS][-1][1] is the second, etc.
    '''

    # returns the plan that achieves the most current goals, based on simulation.
    def get_best_plan(self, world, goals, verbose):
        plan = None
        goalsAchieved = set()
        goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
        for nextPlan in goalGraph.allMatchingPlans(goals):
            achieved = world.goals_achieved(nextPlan, goals)
            if len(achieved) > len(goalsAchieved):
                goalsAchieved = achieved
                plan = nextPlan
            if len(achieved) == len(goals):
                break
            elif verbose >= 2:
                print "Retrieved plan does not achieve all goals. Trying to retrieve a different plan..."
                if verbose >= 3:
                    print "  Retrieved Plan:"
                    for a in nextPlan:
                        print "  " + str(a)
                    print "Goals achieved:", [str(goal) for goal in achieved]
        if plan == None and verbose >= 1:
            print "No valid plan found that achieves any current goals."
        elif len(goalsAchieved) < len(goals) and verbose >= 1:
            print "Best plan does not achieve all goals."
            if verbose >= 2:
                print "Plan:", str(plan)
                print "Goals achieved:", [str(goal) for goal in goalsAchieved]
        return plan

    def run(self, cycle, verbose=2):
        self.verbose = verbose
        max_plan_print_size = 5
        world = self.mem.get(self.mem.STATES)[-1]
        try:
            goals = self.mem.get(self.mem.CURRENT_GOALS)
        except:
            goals = []
        plan = self.get_best_plan(world, goals, verbose)
        trace = self.mem.trace
        if trace:
            trace.add_module(cycle, self.__class__.__name__)
            trace.add_data("WORLD", copy.deepcopy(world))
            trace.add_data("GOALS", copy.deepcopy(goals))
            trace.add_data("PLAN", copy.deepcopy(plan))

        if plan != None:
            action = plan.get_next_step()
            if not action:
                if verbose >= 1:
                    print "Plan to achieve goals has already been completed. Taking no action."
                self.mem.add(self.mem.ACTIONS, [])
            else:
                if verbose == 1:
                    print "Action selected:", action
                elif verbose >= 2:
                    if len(plan) > max_plan_print_size:
                        # print just the next 3 actions of the plan
                        print "Selected action", action, "from plan:\n"
                        if verbose >= 3:
                            for a in plan:
                                print "  " + str(a)
                    else:
                        # print the whole plan
                        print "Selected action", action, "from plan:\n", plan
                self.mem.add(self.mem.ACTIONS, [action])
                actions = self.mem.get(self.mem.ACTIONS)
                if len(actions) > 400:
                    actions = actions[200:]  # trim off old stale actions
                    self.mem.set(self.mem.ACTIONS, actions)
                    # print "Trimmed off 200 old stale actions to save space"
                plan.advance()

                if trace: trace.add_data("ACTION", action)
        else:
            if verbose >= 1:
                print "MIDCA will not select an action this cycle."
            self.mem.add(self.mem.ACTIONS, [])
            if goals:
                for g in goals:
                    self.mem.get(self.mem.GOAL_GRAPH).remove(g)

            if trace: trace.add_data("ACTION", None)

class NBeaconsSimpleAct(base.BaseModule):

    '''
    MIDCA module that selects the plan, if any, that achieves the most current goals, then selects the next action from that plan. The selected action is stored in a two-dimensional array in mem[mem.ACTIONS], where mem[mem.ACTIONS][x][y] returns the yth action to be taken at time step x. So mem[mem.ACTIONS][-1][0] is the last action selected. Note that this will throw an index error if no action was selected.
    To have MIDCA perform multiple actions in one cycle, simple add several actions to mem[mem.ACTIONS][-1]. So mem[mem.ACTIONS][-1][0] is the first action taken, mem[mem.ACTIONS][-1][1] is the second, etc.
    '''

    def get_first_plan(self, goals):
        goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
        plans = goalGraph.allMatchingPlans(goals)
        for p in plans:
            if p.finished():
                goalGraph.removePlan(p)
                if self.verbose >= 1:
                    print "Just removed finished plan "
                    for ps in p:
                        print "  "+str(ps)
            else:
                return p
        if self.verbose >= 1: print "Could not find an unfinished plan in get_first_plan() for goals "+str(goals)
        return None

    def run(self, cycle, verbose = 2):
        self.verbose = verbose
        max_plan_print_size = 10
        world = self.mem.get(self.mem.STATES)[-1]
        try:
            goals = self.mem.get(self.mem.CURRENT_GOALS)[-1]
        except:
            goals = []
        plan = self.get_first_plan(goals)

        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("WORLD", copy.deepcopy(world))
            trace.add_data("GOALS", copy.deepcopy(goals))
            trace.add_data("PLAN", copy.deepcopy(plan))

        if plan != None:
            action = plan.get_next_step()
            if not action:
                if verbose >= 1:
                    print "Plan to achieve goals has already been completed. Taking no action."
                self.mem.add(self.mem.ACTIONS, [])
            else:
                if verbose == 1:
                    print "Action selected:", action
                elif verbose >= 2:
                    if len(plan) > max_plan_print_size:
                        # print just the next 3 actions of the plan
                        print "Selected action", action, "from plan:\n"
                        if verbose >= 3:
                            for a in plan:
                                if action == a:
                                    print "   *"+str(a)
                                else:
                                    print "  "+str(a)
                    else:
                        # print the whole plan
                        print "Selected action", action, "from plan:\n"
                        for a in plan:
                            if action == a:
                                print "   *"+str(a)
                            else:
                                print "  "+str(a)

                self.mem.add(self.mem.ACTIONS, [action])
                plan.advance()

                if trace: trace.add_data("ACTION", action)
        else:
            if verbose >= 1:
                print "MIDCA will not select an action this cycle."
            self.mem.add(self.mem.ACTIONS, [])

            if trace: trace.add_data("ACTION", None)


class Moosact(base.BaseModule):

    def init(self, world, mem):
        context = zmq.Context()
        self.publisher = context.socket(zmq.PUB)
        self.publisher_mine = context.socket(zmq.PUB)
        self.publisher.bind("tcp://127.0.0.1:5560")
        self.publisher_mine.connect("tcp://127.0.0.1:5565")
        self.mem = mem
        self.world = world


    #returns the plan that achieves the most current goals, based on simulation.
    def get_best_plan(self, world, goals, verbose):
        plan = None
        goalsAchieved = set()
        goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
        for nextPlan in goalGraph.allMatchingPlans(goals):
            achieved = world.goals_achieved(nextPlan, goals)
            if len(achieved) > len(goalsAchieved):
                goalsAchieved = achieved
                plan = nextPlan
            if len(achieved) == len(goals):
                break
            elif verbose >= 2:
                print "Retrieved plan does not achieve all goals. Trying to retrieve a different plan..."
                if verbose >= 3:
                    print "  Retrieved Plan:"
                    for a in nextPlan:
                        print "  "+str(a)
                    print "Goals achieved:", [str(goal) for goal in achieved]
        if plan == None and verbose >= 1:
            print "No valid plan found that achieves any current goals."
        elif len(goalsAchieved) < len(goals) and verbose >= 1:
            print "Best plan does not achieve all goals."
            if verbose >= 2:
                print "Plan:", str(plan)
                print "Goals achieved:", [str(goal) for goal in goalsAchieved]
        return plan

    def check_action_completed(self,action):
        for each in action.results:
            if not self.world.atom_true(each):
                return False
        return True

    def execute_action(self,verbose):
        try:
            #get selected actions for this cycle. This is set in the act phase.
            actions = self.mem.get(self.mem.ACTIONS)[-1]
        except TypeError, IndexError:
            if verbose >= 1:
                print "Simulator: no actions selected yet by MIDCA."
            return False

        if actions:
            for action in actions:
                if self.world.midca_action_applicable(action):
                    if (action.op == "avoid"):
                        remus_loc = self.mem.get(self.mem.REMUS_LOCATION)
                        remus_speed = remus_loc['speed']
                        if remus_speed == 0.0 :
                                    self.world.apply_midca_action(action)
                                    self.mem.set(self.mem.MOOS_FEEDBACK, None)
                                    return True

                    # If it's continuing the same action then dont disturb

                    if self.mem.get(self.mem.MOOS_FEEDBACK):
                        if self.world.midca_action_applicable(self.mem.get(self.mem.MOOS_FEEDBACK)) \
                                                and self.mem.get(self.mem.MOOS_FEEDBACK) == action:
                            return False

                        else:
                            self.mem.set(self.mem.MOOS_FEEDBACK, None)


                    if (action.op == "avoid"):
                        label = action.args[0]
                        mines_path = self.mem.get(self.mem.MINE_LOCATION)
                        remus_loc = self.mem.get(self.mem.REMUS_LOCATION)
                        x = mines_path[label]['X']
                        y = mines_path[label]['Y']
                        remus_x = remus_loc['X']
                        remus_y = remus_loc['Y']
                        remus_speed = remus_loc['speed']

                        message = [b"M", b"points ="+str(x+10)+","+str(y)+":"+str(x)+","+str(y-10)+" # speed= 1"]
                        for i in range(2):
                            self.publisher.send_multipart(
                                                [b"M", b"speed =0.0"])

                        suspended_action = self.mem.get(self.mem.MOOS_SUSPENDED_ACTION)
                        if (suspended_action) \
                                and (suspended_action == message):
                            for i in range(2):
                                self.publisher.send_multipart([b"M", b"speed = 1"])

                            self.mem.set(self.mem.MOOS_FEEDBACK, action)
                            return False

                        else:
                            for i in range(2):
                                self.publisher.send_multipart(message)
                            self.mem.set(self.mem.MOOS_FEEDBACK, action)
                            self.mem.set(self.mem.MOOS_SUSPENDED_ACTION, message)
                            return False


                    elif (action.op == "ignore"):
                        label = int(action.args[0].replace("mine", ""))
                        for i in range(2):
                            self.publisher.send_multipart(
                                                [b"M", b"speed =0.0"])
                        self.publisher.send_multipart(
                                                [b"M", b"speed = 0.0001"])
                        self.world.apply_midca_action(action)
                        self.mem.set(self.mem.MOOS_FEEDBACK, None)
                        return True

                    elif (action.op == "remove_mines"):
                        self.world.apply_midca_action(action)
                        self.mem.set(self.mem.MOOS_FEEDBACK, None)
                        return True

                    elif (action.op == "remove"):
                        label= int(action.args[0].replace("mine",""))

                        if (action.args[1] == "ga1" or action.args[1] == "ga2"):
                            score = self.mem.get(self.mem.MOOS_SCORE) + 1
                            self.mem.set(self.mem.MOOS_SCORE, score)

                        if (action.args[1] == "qroute"):
                            score = self.mem.get(self.mem.MOOS_SCORE) + 1
                            self.mem.set(self.mem.MOOS_SCORE, score)

                        for i in range(2):
                            self.publisher.send_multipart(
                            [b"M", b"speed =0.0"])
                            self.publisher_mine.send_multipart(
                                                [b"M", str(label)])
                        self.publisher.send_multipart(
                                                [b"M", b"speed = 0.0001"])
                        time.sleep(2)
                        self.world.apply_midca_action(action)
                        self.mem.set(self.mem.MOOS_FEEDBACK, None)

                        return True

                    elif (action.op == "fast_survey"):
                        argnames = [str(arg) for arg in action.args]

                        if ("transit1" in argnames):
                            message = [b"M", b"point = 0,-20 # speed= 1.0"]
                            suspended_action = self.mem.get(self.mem.MOOS_SUSPENDED_ACTION)
                            # if the action gets suspended then just give the moos the speed
                            # because it will have it's behaviour already running
                            if (suspended_action) \
                                    and (suspended_action == message):
                                for i in range(2):
                                    self.publisher.send_multipart([b"M", b"speed = 1.0"])
                                self.mem.set(self.mem.MOOS_FEEDBACK, action)
                                return False

                            else:
                                for i in range(2):
                                    self.publisher.send_multipart(message)
                                self.mem.set(self.mem.MOOS_FEEDBACK, action)
                                self.mem.set(self.mem.MOOS_SUSPENDED_ACTION, message)
                                return False

                        if ("qroute_transit" in argnames):
                            message = [b"M", b"point = 60,-65 # speed= 1.0"]
                            suspended_action = self.mem.get(self.mem.MOOS_SUSPENDED_ACTION)
                            if (suspended_action) \
                                    and (suspended_action == message):
                                for i in range(2):
                                    self.publisher.send_multipart([b"M", b"speed = 1.0"])
                                self.mem.set(self.mem.MOOS_FEEDBACK, action)
                                return False

                            else:
                                for i in range(2):
                                    self.publisher.send_multipart(message)
                                self.mem.set(self.mem.MOOS_FEEDBACK, action)
                                self.mem.set(self.mem.MOOS_SUSPENDED_ACTION, message)
                                return False


                        if ("home" in argnames):
                            message = [b"M", b"point = 167,0 # speed= 1.0"]
                            suspended_action = self.mem.get(self.mem.MOOS_SUSPENDED_ACTION)
                            if (suspended_action) \
                                    and (suspended_action == message):
                                for i in range(2):
                                    self.publisher.send_multipart([b"M", b"speed = 1.0"])
                                self.mem.set(self.mem.MOOS_FEEDBACK, action)
                                return False

                            else:
                                for i in range(2):
                                    self.publisher.send_multipart(message)
                                self.mem.set(self.mem.MOOS_FEEDBACK, action)
                                self.mem.set(self.mem.MOOS_SUSPENDED_ACTION, message)

                                return False

                        if ("transit2" in argnames):
                            message = [b"M", b"point = 154,-27 # speed= 0.5"]
                            suspended_action = self.mem.get(self.mem.MOOS_SUSPENDED_ACTION)
                            if (suspended_action) \
                                    and (suspended_action == message):
                                for i in range(2):
                                    self.publisher.send_multipart([b"M", b"speed = 0.5"])
                                self.mem.set(self.mem.MOOS_FEEDBACK, action)
                                return False

                            else:
                                for i in range(2):
                                    self.publisher.send_multipart(message)
                                self.mem.set(self.mem.MOOS_FEEDBACK, action)
                                self.mem.set(self.mem.MOOS_SUSPENDED_ACTION, message)

                                return False

                    elif (action.op == "slow_survey"):
                        argnames = [str(arg) for arg in action.args]

                        if ("ga1" in argnames):
                            message = [b"M",b" points=format=lawnmower,label=dedley_survey, x=20, y=-80, width=20, height = 30,lane_width=20, rows=north-south,degs=0 # speed =0.5"]
                            suspended_action = self.mem.get(self.mem.MOOS_SUSPENDED_ACTION)

                            if (suspended_action) \
                                and (suspended_action == message):
                                print ("Continue Surveying ....")
                                for i in range(2):
                                    self.publisher.send_multipart([b"M", b"speed =0.5"])
                                self.mem.set(self.mem.MOOS_FEEDBACK, action)
                                return False

                            else:
                                print ("Start Surveying ....")
                                for i in range(2):
                                    self.publisher.send_multipart(message)
                                self.mem.set(self.mem.MOOS_FEEDBACK , action)
                                self.mem.set(self.mem.MOOS_SUSPENDED_ACTION, message)
                                return False

                        if ("ga2" in argnames):
                            message = [b"M",b" points=format=lawnmower,label=dedley_survey, x=150, y=-80, width=20, height = 30,lane_width=20, rows=north-south,degs=0 # speed =0.5"]
                            suspended_action = self.mem.get(self.mem.MOOS_SUSPENDED_ACTION)
                            if (suspended_action) \
                                    and (suspended_action == message):
                                for i in range(2):
                                    self.publisher.send_multipart([b"M", b"speed = 0.5"])
                                self.mem.set(self.mem.MOOS_FEEDBACK, action)
                                return False

                            else:
                                for i in range(2):
                                    self.publisher.send_multipart(message)
                                self.mem.set(self.mem.MOOS_FEEDBACK, action)
                                self.mem.set(self.mem.MOOS_SUSPENDED_ACTION, message)
                                return False

                        if ("home" in argnames):
                            message = [b"M", b"point = 170,0 # speed= 0.5"]
                            suspended_action = self.mem.get(self.mem.MOOS_SUSPENDED_ACTION)
                            if (suspended_action) \
                                    and (suspended_action == message):
                                for i in range(2):
                                    self.publisher.send_multipart([b"M", b"speed = 0.5"])
                                self.mem.set(self.mem.MOOS_FEEDBACK, action)
                                return False

                            else:
                                for i in range(2):
                                    self.publisher.send_multipart(message)
                                self.mem.set(self.mem.MOOS_FEEDBACK, action)
                                self.mem.set(self.mem.MOOS_SUSPENDED_ACTION, message)

                                return False

                else:
                    if verbose >= 1:
                        print "MIDCA-selected action", action, "illegal in current world state. Skipping"
                    return True

        return False

    def run(self, cycle, verbose = 2):
        self.verbose = verbose
        max_plan_print_size = 5
        world = self.mem.get(self.mem.STATES)[-1]

        try:
            goals = self.mem.get(self.mem.CURRENT_GOALS)[-1]
        except :
            goals = []

        plan = self.get_best_plan(world, goals, verbose)
        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("WORLD", copy.deepcopy(world))
            trace.add_data("GOALS", copy.deepcopy(goals))
            trace.add_data("PLAN", copy.deepcopy(plan))

        if plan != None:
            action = plan.get_next_step()
            if not action:
                if verbose >= 1:
                    print "Plan to achieve goals has already been completed. Taking no action."
                self.mem.add(self.mem.ACTIONS, [])
            else:
                if verbose == 1:
                    print "Action selected:", action
                elif verbose >= 2:
                    if len(plan) > max_plan_print_size:
                        # print just the next 3 actions of the plan
                        print "Selected action", action, "from plan:\n"
                        if verbose >= 3:
                            for a in plan:
                                print "  "+str(a)
                    else:
                        # print the whole plan
                        print "Selected action", action, "from plan:\n", plan
                self.mem.add(self.mem.ACTIONS, [action])
                actions = self.mem.get(self.mem.ACTIONS)
                if len(actions) > 400:
                    actions = actions[200:] # trim off old stale actions
                    self.mem.set(self.mem.ACTIONS, actions)
                    #print "Trimmed off 200 old stale actions to save space"
                if self.execute_action(verbose):
                    plan.advance()

                if trace: trace.add_data("ACTION", action)
        else:
            if verbose >= 1:
                print "MIDCA will not select an action this cycle."
            self.mem.add(self.mem.ACTIONS, [])
            if goals:
                for g in goals:
                    self.mem.get(self.mem.GOAL_GRAPH).remove(g)

            if trace: trace.add_data("ACTION", None)
