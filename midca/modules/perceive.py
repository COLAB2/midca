from midca.modules._robot_world import world_repr
from midca import rosrun, midcatime, base, goals
from midca.worldsim import domainread, stateread
import copy
import os
import socket, zmq
try:
	# baxter robot requirements
	from midca.examples import ObjectDetector
	from bzrlib.config import LocationStore
except:
	pass

class ROSObserver:
    
    def init(self, world, mem):
        self.mem = mem
        self.mem.set(self.mem.STATE, world_repr.SimpleWorld())

    def store_history(self,world,history,blocks):
	'''
	store the history of last 5 state changes
	'''
	if blocks:
		a = {}
		for each in blocks:
			positions= world.all_pos(each)
			a[each] = positions.pop().position

		if a:
			history = history.append(a)

		if not history:
			history = []

		if len(history) > 5:
			history = history[:5]

		history.reverse()
		return history
	return None
	

    

    def check_with_history(self,world,history,detectionEvents):
	'''
	store the past 5 change in events for the robot to remember things
	'''
	blocks = set()
	for each in detectionEvents:
		blocks.add(each.id)
	if not history:
		history = []
		self.store_history(world,history,blocks)
	else:
		if not len(blocks) == len(history[len(history) -1]):
			history = self.store_history(world,history,blocks)
	return history
    
    def run(self, cycle, verbose = 2):
        #self.ObserveWorld() 
        detectionEvents = self.mem.get_and_clear(self.mem.ROS_OBJS_DETECTED)
        detecttionBlockState = self.mem.get_and_clear(self.mem.ROS_OBJS_STATE)
        utteranceEvents = self.mem.get_and_clear(self.mem.ROS_WORDS_HEARD)
        feedback = self.mem.get_and_clear(self.mem.ROS_FEEDBACK)
        world = self.mem.get_and_lock(self.mem.STATE)
	history = self.mem.get_and_lock(self.mem.STATE_HISTORY)

        if not detectionEvents:
            detectionEvents = []
        if not detecttionBlockState:
            detecttionBlockState = []
        if not utteranceEvents:
            utteranceEvents = []
        if not feedback:
            feedback = []
        for event in detectionEvents:
            event.time = midcatime.now()
            world.sighting(event)
        for blockstate in detecttionBlockState:
            blockstate.time = midcatime.now()
            world.position(blockstate)
        for event in utteranceEvents:
            event.time = midcatime.now()
            world.utterance(event)
        for msg in feedback:
            d = rosrun.msg_as_dict(msg)
            d['received_at'] = float(midcatime.now())
            self.mem.add(self.mem.FEEDBACK, d)

	# if there are any change in events remember
	history = self.check_with_history(world,history,detecttionBlockState)
	self.mem.unlock(self.mem.STATE_HISTORY)
	if history:
		if len(history) > 5:
			history = history[:5]
		self.mem.set(self.mem.STATE_HISTORY , history)		
        self.mem.unlock(self.mem.STATE)


        if verbose > 1:
            print "World observed:", len(detectionEvents), "new detection event(s),", len(utteranceEvents), "utterance(s) and", len(feedback), "feedback msg(s)"
            
    

class PerfectObserver(base.BaseModule):

    '''
    MIDCA Module which copies a complete world state. It is designed to interact with the
    built-in MIDCA world simulator. To extend this to work with other representations,
    modify the observe method so that it returns an object representing the current known
    world state.
    '''

    def init(self, world, mem):
        base.BaseModule.init(self, mem)
        if not world:
            raise ValueError("world is None!")
        self.world = world

    #perfect observation
    def observe(self):
        return self.world.copy()

    def run(self, cycle, verbose = 2):
        world = self.observe()
        print (world)
        if not world:
            raise Exception("World observation failed.")
        self.mem.add(self.mem.STATES, world)
        
        # Memory Usage Optimization (optional, feel free to comment
        # drop old memory states if not being used
        # this should help with high memory costs
        states = self.mem.get(self.mem.STATES)
        if len(states) > 400:
            #print "trimmed off 200 old stale states"
            states = states[200:]
            self.mem.set(self.mem.STATES, states)
        # End Memory Usage Optimization
        
        if verbose >= 1:
            print "World observed."
        
        trace = self.mem.trace
        if trace:
            trace.add_module(cycle, self.__class__.__name__)
            trace.add_data("WORLD",copy.deepcopy(world))

class RecieveRequests(base.BaseModule):

    '''
    MIDCA Module which copies a complete world state. It is designed to interact with the
    built-in MIDCA world simulator. To extend this to work with other representations,
    modify the observe method so that it returns an object representing the current known
    world state.
    '''
    def __init__(self, publish, subscribe, name , other_agent_name):
        self.name = name
        self.other_agent_name = other_agent_name

        context = zmq.Context()
        self.publisher = context.socket(zmq.PUB)
        self.publisher.bind(publish)

        context = zmq.Context()
        self.subscriber = context.socket(zmq.SUB)
        self.subscriber.setsockopt(zmq.RCVTIMEO, 5)
        self.subscriber.setsockopt(zmq.SUBSCRIBE, "")
        self.subscriber.connect(subscribe)


    def init(self, world, mem):
        base.BaseModule.init(self, mem)
        if not world:
            raise ValueError("world is None!")
        self.world = world
        self.mem.set(self.mem.CONNECTIONS, {"publish" : self.publisher,
                                                        "subscribe": self.subscriber})

    def goal_to_parsable_construct(self, goal):
        """
        :param goal: goal
        :return: to a string that can be read by stateread
        """
        # convert commas in goal to ":" because of problem in parsing predicates by stateread
        goal = str(goal).replace(",",";")
        # convert round braces to square braces because of problem in parsing predicates by stateread
        goal = goal.replace("(","[")
        goal = goal.replace(")","]")

        return goal

    def run(self, cycle, verbose = 2):
        states = ""
        # get the message from the ip address
        try:
            message = self.subscriber.recv()
            message = message.split(":")
            performative = message[0]
            goaltext = message[1]

            if performative == "achieve":
                print "Request Recieved."
                goal = self.goal_to_parsable_construct(goaltext)
                states += "GOAL(" +goal + ")\n"
                states += "requested(" + self.other_agent_name + "," + self.name+ "," +goal+")\n"

            elif performative == "reject":
                print "Request Recieved."
                goal = self.goal_to_parsable_construct(goaltext)
                states += "GOAL(" +goal + ")\n"
                states += "rejected(" + self.other_agent_name + "," + self.name+ "," +goal+")\n"

            elif performative == "commit":
                print "Request Recieved."
                goal = self.goal_to_parsable_construct(goaltext)
                states += "GOAL(" +goal + ")\n"
                states += "committed(" + self.other_agent_name + "," + self.name+ "," +goal+")\n"

            elif performative == "tell":
                print "Request Recieved."
                goal = self.goal_to_parsable_construct(goaltext)
                states += "GOAL(" +goal + ")\n"
                states += "!committed(" + self.other_agent_name + "," + self.name+ "," +goal+")\n"
                states += "achieved(" + self.other_agent_name + "," + self.name+ "," +goal+")\n"


                    # this is to update the world into memory
            if not states == "":
                if verbose >= 1:
                    print ("Updated States \n")
                    print(states)
                stateread.apply_state_str(self.world, states)
                self.mem.add(self.mem.STATES, self.world)

            # trace
            trace = self.mem.trace
            if trace:
                trace.add_module(cycle, self.__class__.__name__)
                trace.add_data("WORLD",copy.deepcopy(self.world))

        except zmq.ZMQError as e:
            if e.errno == zmq.EAGAIN:
                pass # no message was ready (yet!)

class RecieveRemoteMidcaWorld(base.BaseModule):

    '''
    MIDCA Module which copies a complete world state. It is designed to interact with the
    built-in MIDCA world simulator. To extend this to work with other representations,
    modify the observe method so that it returns an object representing the current known
    world state.
    '''
    def __init__(self, publish, subscribe):

        context = zmq.Context()
        self.subscriber = context.socket(zmq.PULL)
        self.subscriber.setsockopt(zmq.RCVTIMEO, -1)
        self.subscriber.setsockopt(zmq.CONFLATE, 1)
        self.subscriber.connect(subscribe)



    def init(self, world, mem):
        base.BaseModule.init(self, mem)
        if not world:
            raise ValueError("world is None!")
        self.world = world

    def run(self, cycle, verbose = 2):
        # get the message from the ip address
        try:
            world = self.subscriber.recv_pyobj()
            world = world.copy()
            #recv_world_atoms, world_atoms = world.diff(self.world)
            #for atom in world_atoms:
            #    world.add_atom(atom)
            copy_world = self.world.copy()
            predicates = ["informed", "requested", "committed",
                          "achieved", "exists", "rejected"]
            for atom in copy_world.atoms:
                if not atom.predicate.name in predicates:
                    self.world.remove_atom(atom)

            for atom in world.atoms:
                self.world.add_fact(atom.predicate.name, [arg.name for arg in atom.args])

        except zmq.ZMQError as e:
            if e.errno == zmq.EAGAIN:
                pass # no message was ready (yet!)


class PerfectObserverWithThief(base.BaseModule):

    '''
    MIDCA Module which copies a complete world state. It is designed to interact with the
    built-in MIDCA world simulator. To extend this to work with other representations,
    modify the observe method so that it returns an object representing the current known
    world state.
    '''

    def init(self, world, mem):
        base.BaseModule.init(self, mem)
        if not world:
            raise ValueError("world is None!")
        self.world = world

    #perfect observation
    def observe(self):
        return self.world.copy()
	
    def run(self, cycle, verbose = 2):
        world = self.observe()
        thisDir = os.path.dirname(os.path.realpath(__file__))
        thief_file = thisDir + "/theif.txt"
        theft_items=[]
        
#         with open(thief_file) as f:
# 	    	lines = f.readlines()
# 	    	for line in lines:
# 	    		theft_items.append(line.split(" "))
# 	    	
        if not world:
            raise Exception("World observation failed.")
        
#         self.mem.add(self.mem.STATES, world)
        
        for item in theft_items:
        	
			for atom in world.atoms:
				if atom.predicate.name == item[0] and atom.args[0].name == item[1]:
					world.atoms.remove(atom)   
					print("removed:" + atom.args[0].name)
					break
         			
        self.mem.add(self.mem.STATES, world) 
        
        # Memory Usage Optimization (optional, feel free to comment
        # drop old memory states if not being used
        # this should help with high memory costs
        states = self.mem.get(self.mem.STATES)
        if len(states) > 400:
            #print "trimmed off 200 old stale states"
            states = states[200:]
            self.mem.set(self.mem.STATES, states)
        # End Memory Usage Optimization
        
        if verbose >= 1:
            print "World observed."
        
        trace = self.mem.trace
        if trace:
            trace.add_module(cycle, self.__class__.__name__)
            trace.add_data("WORLD",copy.deepcopy(world))
        
class UserGoalInput(base.BaseModule):

    '''
    MIDCA module that allows users to input goals in a predicate representation.
    These will be stored in MIDCA state
    Note that this class only allows for simple goals with only predicate and argument information.
    It does not currently check to see whether the type or number of arguments is appropriate.
    '''
    def __init__(self, name):
        self.name = name

    def init(self, world, mem):
        base.BaseModule.init(self, mem)
        if not world:
            raise ValueError("world is None!")
        self.world = world

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

    def objectNames(self, world):
        return world.objects.keys()

    def predicateNames(self, world):
        return world.predicates.keys()

    def validGoal(self, goal, world):
        try:
            for arg in goal.args:
                if arg not in self.objectNames(world):
                    return False
            return goal['predicate'] in self.predicateNames(world)
        except Exception:
            return False

    def goal_to_parsable_construct(self, goal):
        """
        :param goal: goal
        :return: to a string that can be read by stateread
        """
        # convert commas in goal to ":" because of problem in parsing predicates by stateread
        goal = str(goal).replace(",",";")
        # convert round braces to square braces because of problem in parsing predicates by stateread
        goal = goal.replace("(","[")
        goal = goal.replace(")","]")

        return goal

    def run(self, cycle, verbose = 2):
        if verbose == 0:
            return #if skipping, no user input
        goals_entered = []
        states = ""
        while True:
            val = raw_input("Please input a goal if desired. Otherwise, press enter to continue\n")
            if not val:
                break
            elif val == 'q':
                break
            goaltext = val.strip()
            goal = self.parseGoal(val.strip())
            if goal:
                if not self.validGoal(goal, self.world):
                    print str(goal), "is not a valid goal\nPossible predicates:", self.predicateNames(self.world), "\nPossible arguments", self.objectNames(self.world)
                else:
                    #self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                    print "Goal Recieved."
                    goal = self.goal_to_parsable_construct(goaltext)
                    states += "exists(human)\n"
                    states += "GOAL(" +goal + ")\n"
                    states += "requested(human," +self.name+ "," +goal+")\n"

        # this is to update the world into memory
        if not states == "":
            if verbose >= 1:
                print ("Updated States \n")
                print(states)
            stateread.apply_state_str(self.world, states)
            self.mem.add(self.mem.STATES, self.world)

        # trace
        trace = self.mem.trace
        if trace:
            trace.add_module(cycle, self.__class__.__name__)
            trace.add_data("WORLD",copy.deepcopy(self.world))

class MAReport:

    namecounts = {"report": 0}

    def __init__(self):
        self.actions = []
        self.finalstate = None

    def str_dict(self, item, numtabs = 1, skipfirsttab = True):
        if isinstance(item, dict):
            s = ""
            first = True
            for key in sorted(item.keys()):
                if not first or not skipfirsttab:
                    s += "\t" * numtabs
                else:
                    s += " "
                s += "(" + str(key) + " " + self.str_dict(item[key], numtabs + 1) + ")\n"
                first = False
            s = s[:-1]
        else:
            s = str(item)
        return s

    def action_str(self, action):
        if action[0] in self.namecounts:
            self.namecounts[action[0]] += 1
        else:
            self.namecounts[action[0]] = 1
        s = "(" + str(action[0]) + "." + str(self.namecounts[action[0]]) + "\n"
        valuepairs = {}
        if action[0] in ("stack", "unstack", "pickup", "putdown", "apprehend", "putoutfire"):
            valuepairs["actor"] = {"value": "person.0"}
        elif action[0] == "burns":
            valuepairs["actor"] = {"value": "nature"}
        valuepairs["object"] = {"value": str(action[1]).replace(" ", "_")}
        if action[0] in ("stack", "unstack"):
            valuepairs["recipient"] = {"value": str(action[2]).replace(" ", "_")}
        return s + self.str_dict(valuepairs, skipfirsttab = False) + ")"

    def atom_pairs(self, atom):
        valuepairs = {}
        for i in range(len(atom.args)):
            if i == 0:
                valuepairs["domain"] = {"value": atom.args[i].name.replace(" ", "_")}
            elif i == 1:
                valuepairs["co-domain"] = {"value": atom.args[i].name.replace(" ", "_")}
        return valuepairs

    def state_str(self, world):
        s = "(state\n"
        valuepairs = {}
        for atom in world.atoms:
            if atom.predicate.name in self.namecounts:
                self.namecounts[atom.predicate.name] += 1
            else:
                self.namecounts[atom.predicate.name] = 1
            valuepairs[atom.predicate.name + "." + str(self.namecounts[atom.predicate.name])] = self.atom_pairs(atom)
        return s + self.str_dict(valuepairs, skipfirsttab = False) + ")"


    def __str__(self):
        #if not self.actions:
        #    return "incomplete"
        # if there is no state do not send report to meta aqua
        if not self.finalstate:
            return "incomplete"
        else:
            self.namecounts["report"] += 1
            s = "(" + "report." + str(self.namecounts["report"]) + "\n("
            for action in self.actions:
                s += "\t(\n"
                s += self.action_str(action)
                s += "\n\"\")\n("
                s += self.state_str(self.finalstate)
                s += "\n\"\")\n"
            return s + "))"

'''
ma = MAReport()
import domainread, stateread
world = domainread.load_domain("./domain.sim")
stateread.apply_state_file(world, "./defstate.sim")
ma.finalstate = world
ma.actions.append(["unstack", "block1", "block2"])
ma.actions.append(["catchfire", "block1"])
print ma
'''

class MAReporter(base.BaseModule):

    '''
    MIDCA module that sends a report on the world and actions to the
    Meta-AQUA story understanding system. This requires Meta-AQUA to be
    running or it will not work. Also depends on the basic observation
    module.
    '''

    def __init__(self, writePort):
        self.writeS = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.writeS.connect(("localhost", writePort))

    def get_lit_blocks(self, world):
        res = []
        for objectname in world.objects:
            if world.is_true("onfire", [objectname]) and \
            world.objects[objectname].type.name == "BLOCK" and \
            objectname != "table":
                res.append(objectname)
        return res
    

    def run(self, cycle, verbose = 2):
        world = None
        lastWorld = None
        try:
            world = self.mem.get(self.mem.STATES)[-1]
            lastWorld = self.mem.get(self.mem.STATES)[-2]
        except (TypeError,IndexError):
            pass
        if not world:
            return #no report if not world observed
        report = MAReport()
        report.finalstate = world
        try:
            actions = self.mem.get(self.mem.ACTIONS)[-1]
        except (TypeError, IndexError):
            actions = []
        blocksPutOut = []
        for action in actions:
            action.args = list(action.args)
            report.actions.append([action.op] + action.args)
            if action.op == "putoutfire":
                blocksPutOut.append(action.args[0])
        if lastWorld:
            lastBurning = self.get_lit_blocks(lastWorld)
            burning = self.get_lit_blocks(world)
            for block in burning:
                if block not in lastBurning or block in blocksPutOut:
                    report.actions.append(["burns", block])
        #report is finished, send to Meta-AQUA
		#report contains actions and state, 
		#for every action there will be the state attached to it
        if verbose >= 1:
            print "Sending report to Meta-AQUA",
            if verbose >= 2:
                print ":\n", report
        if not str(report ) == "incomplete":
            self.writeS.send(str(report))

    def __del__(self):
        '''
            close sockets on deletion. Also send 'Done' message to Meta-AQUA.
        '''
        try:
            self.writeS.send(self.endMsg)
        finally:
            self.writeS.shutdown(socket.SHUT_RDWR)
