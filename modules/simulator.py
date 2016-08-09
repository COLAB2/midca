import sys, random
from MIDCA import worldsim, goals, base
import copy 

class MidcaActionSimulator:

    def init(self, world, mem):
        self.mem = mem
        self.world = world

    def run(self, cycle, verbose = 2):
        try:
            #get selected actions for this cycle. This is set in the act phase.
            actions = self.mem.get(self.mem.ACTIONS)[-1]
        except TypeError, IndexError:
            if verbose >= 1:
                print "Simulator: no actions selected yet by MIDCA."
            return
        if actions:
            for action in actions:
                if self.world.midca_action_applicable(action):
                    if verbose >= 2:
                        print "simulating MIDCA action:", action
                    self.world.apply_midca_action(action)
                else:
                    if verbose >= 1:
                        print "MIDCA-selected action", action, "illegal in current world state. Skipping"
        else:
            if verbose >= 2:
                print "No actions selected this cycle by MIDCA."

ARSONIST_VICTORY_ACTIVITIES = ["enjoys a glass of champagne", "stays home", "bites his thumb at MIDCA"]

class ASCIIWorldViewer(base.BaseModule):

    def __init__(self, display=None):
        self.display = display
    
    def init(self, world, mem):
        self.world = world

    def run(self, cycle, verbose = 2):
        if verbose >= 2:
            if self.display:
                self.display(self.world)


class WorldChanger:

    def init(self, world, mem):
        self.world = world

    def parseGoal(self, txt):
        if not txt.endswith(")"):
            print "Error reading input. Atom must be given in the form: predicate(arg1, arg2,...,argi-1,argi), where each argument is the name of an object in the world"
            return None
        try:
            predicateName = txt[:txt.index("(")]
            args = [arg.strip() for arg in txt[txt.index("(") + 1:-1].split(",")]
            goal = goals.Goal(*args, predicate = predicateName)
            return goal
        except Exception:
            print "Error reading input. Atom must be given in the form: predicate(arg1, arg2,...,argi-1,argi), where each argument is the name of an object in the world"
            return None

    def run(self, cycle, verbose = 2):
        if verbose == 0:
            return
        while True:
            val = raw_input("If you wish to change the state, please input the desired atom to flip. Otherwise, press enter to continue\n")
            if not val:
                return "continue"
            elif val == 'q':
                return val
            goal = self.parseGoal(val.strip())
            if goal:
                try:
                    atom = self.world.midcaGoalAsAtom(goal)
                    if self.world.atom_true(atom):
                        self.world.remove_atom(atom)
                        print "Atom", atom, "was true and is now false"
                    else:
                        self.world.add_atom(atom)
                        print "Atom", atom, "was false and is now true"
                except ValueError:
                    print "The value entered does not appear to be a valid atom. Please check the number and type of arguments."

class ArsonSimulator:

    def __init__(self, arsonChance = 0.5, arsonStart = 10):
        self.chance = arsonChance
        self.start = arsonStart

    def getArsonChance(self):
        return self.chance

    def init(self, world, mem):
        self.mem = mem
        self.world = world

    def free_arsonist(self):
        for atom in self.world.atoms:
            if atom.predicate.name == "free":
                if atom.args[0].type.name == "ARSONIST":
                    return atom.args[0].name
        return False

    def get_unlit_blocks(self):
        res = []
        for objectname in self.world.objects:
            if not self.world.is_true("onfire", [objectname]) and self.world.objects[objectname].type.name == "BLOCK" and objectname != "table":
                res.append(objectname)
        return res

    def run(self, cycle, verbose = 2):
        arsonist = self.free_arsonist()
        if arsonist and cycle > self.start and random.random() < self.chance:
            try:
                block = random.choice(self.get_unlit_blocks())
                try:
                    self.world.apply_named_action("lightonfire", [arsonist, block])
                    if verbose >= 2:
                        print "Simulating action: lightonfire(" + str(arsonist) + ", " + str(block) + ")"
                except Exception:
                    if verbose >= 1:
                        print "Action lightonfire(", str(arsonist), ",", str(block), ") invalid."
            except IndexError:
                if verbose >= 1:
                    print "All blocks on fire.", arsonist, random.choice(ARSONIST_VICTORY_ACTIVITIES)

SCORE = "Score"

class FireReset:

    '''
    MIDCA module that puts out all fires whenever MIDCA's score is updated to indicate that a tower has been completed. Note that this module will do nothing unless the the SCORE memory value is being updated by evaluate.Scorer.
    '''

    def init(self, world, mem):
        self.world = world
        self.mem = mem
        self.numTowers = 0

    def put_out_fires(self):
        self.world.atoms = [atom for atom in self.world.atoms if atom.predicate.name != "onfire"]

    def run(self, cycle, verbose = 2):
        score = self.mem.get(SCORE)
        if not score:
            return
        if score.towers == self.numTowers:
            return
        self.numTowers = score.towers
        if verbose >= 2:
            print "Since a tower was just completed, putting out all fires."
        self.put_out_fires()


class NBeaconsSimulator:
    '''
    Performs changes to the nbeacons domain, such as:
    1. beacons becoming deactivated
    '''

    def __init__(self, beacon_fail_rate=1):
        '''
        beacon_fail_rate is out of 100. So 100 means 100% chance, 5 means 5% chance
        '''
        self.beacon_fail_rate = beacon_fail_rate

    def init(self, world, mem):
        self.mem = mem
        self.world = world

    #def moverightright(self):

    def run(self, cycle, verbose = 2):
        # deactivate beacons according to fail rate
        world = None
        try:
            world = self.mem.get(self.mem.STATES)[-1]
        except:
            # probably failing on first try, just return and do nothing
            return
        # get all activated beacon ids
        activated_b_ids = []
        for obj in world.get_possible_objects("",""):
            # test if a beacon id
            if str(obj).startswith("B"):
                # now test to see if it's activated
                if world.is_true('activated',[str(obj)]):
                    activated_b_ids.append(str(obj))

        # for each beacon, run the fail rate
        for b_id in activated_b_ids:
            if random.choice(range(100)) < self.beacon_fail_rate:
                self.world.apply_named_action("deactivatebeacon", [b_id])
                if verbose >= 0:
                        print "Simulating action: deactivatebeacon(" + str(b_id) + ")"

class NBeaconsActionSimulator:
    '''
    Performs changes to the midca state specific to NBeacons.
    '''
    
    def __init__(self, wind=False, wind_dir=None, dim=10):
        self.wind = wind
        self.wind_dir = wind_dir
        self.dim = dim
        if self.wind and not self.wind_dir in ['east','west','north','south']:
            raise Exception("Turning wind on requires a wind direction of "+str(['east','west','north','south']))
    
    def init(self, world, mem):
        self.mem = mem
        self.world = world

    def get_subsequent_action(self,action):
        '''
        Does not return a midcaAction, instead just returns an array
        of the values that should be given to world.apply_named_action()
        '''
        subsequent_loc = None
        subsequent_action = None
        if self.wind_dir == 'east':
            # first check to see if the agent is in the right most tile
            agent_loc = self.world.get_atoms(filters=["agent-at"])[0].args[1]
            print "agent_loc is "+str(agent_loc)
            print "about to get_atoms with (filters=[adjacent-east,"+str(agent_loc)+"]"
            for atom in self.world.get_atoms(filters=["adjacent-east",str(agent_loc)]):
                if atom.args[0] == agent_loc:
                    subsequent_loc = atom.args[1]
                    subsequent_action = copy.deepcopy(action)
                    self.mem.get(self.mem.STATES)[-1]
                    print "previous action is " + str(subsequent_action)+" and of type "+str(type(subsequent_action))
                    new_action_op = action.op
                    new_action_args = [action.args[0],str(agent_loc),str(subsequent_loc)]
                    subsequent_action = [new_action_op]+new_action_args
                    print "new_action is "+str([new_action_op]+new_action_args)
                   
                    
        return subsequent_action
            
# 
# def midca_action_applicable(self, midcaAction):
#         try:
#             operator = self.operators[midcaAction.op]
#             args = [self.objects[arg] for arg in midcaAction.args]
#         except KeyError:
#             return False
#         action = operator.instantiate(args)
#         return self.is_applicable(action)

    def run(self, cycle, verbose = 2):
        try:
            #get selected actions for this cycle. This is set in the act phase.
            actions = self.mem.get(self.mem.ACTIONS)[-1]
        except TypeError, IndexError:
            if verbose >= 1:
                print "Simulator: no actions selected yet by MIDCA."
            return
        if actions:
            for action in actions:
                if self.world.midca_action_applicable(action):
                    if verbose >= 2:
                        print "simulating MIDCA action:", action
                    self.world.apply_midca_action(action)
                    
                    if self.wind and self.wind_dir in str(action):
                        # duplicate the effect because wind is pushing the agent
                        subseq_action = self.get_subsequent_action(action)
                        self.world.apply_named_action(subseq_action[0],subseq_action[1:])
                else:
                    if verbose >= 1:
                        print "MIDCA-selected action", action, "illegal in current world state. Skipping"
        else:
            if verbose >= 2:
                print "No actions selected this cycle by MIDCA."


class CustomRunSimulator:
    '''
    This class is used to provide various kinds of commands to be used in customrun xml files (files that make it easier to run MIDCA experiments and control inputs, outputs, etc). For more information about using customrun config files, see customrun/customrun.py. Feel free to add any functions you need here.
    '''

    def writeDataToCSV(self,filename):
        ''' '''
        pass




