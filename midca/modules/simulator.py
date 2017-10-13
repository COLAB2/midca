import sys, random
from midca import worldsim, goals, base
from midca.domains.nbeacons import nbeacons_util
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

    def __init__(self, beacon_fail_rate=0):
        '''
        beacon_fail_rate is out of 100. So 100 means 100% chance, 5 means 5% chance.
        Note this is per beacon, so each beacon has the given chance of failing
        '''
        self.beacon_fail_rate = beacon_fail_rate

    def init(self, world, mem):
        self.mem = mem
        self.world = world

    def run(self, cycle, verbose = 2):
        self.verbose = verbose
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
                if self.verbose >= 1:
                    print "Simulating action: deactivatebeacon(" + str(b_id) + ")"

class NBeaconsActionSimulator:
    '''
    Performs changes to the midca state specific to NBeacons.
    '''
    
    def __init__(self, wind=False, wind_dir='off', wind_strength=1, dim=10, wind_schedule=[],logfilenm=""):
        self.logfilenm = logfilenm
        self.filehandle = None
        if len(self.logfilenm) > 0:
            self.filehandle = open(self.logfilenm, 'w')
        self.wind_schedule = wind_schedule
        self.wind = wind
        self.wind_dir = wind_dir
        self.wind_strength = wind_strength
        self.dim = dim
        if self.wind and not self.wind_dir in ['east','west','north','south']:
            self.wind = False
            self.wind_strength = 0
            #raise Exception("Turning wind on requires a wind direction of "+str(['east','west','north','south']))
    
    def init(self, world, mem):
        self.mem = mem
        self.world = world
        self.mem.set(self.mem.MIDCA_CYCLES, 0)

    def get_subsequent_action(self,action,dir):
        '''
        Does not return a midcaAction, instead just returns an array
        of the values that should be given to world.apply_named_action()
        '''
        subsequent_action = None
        
        prev_action_dest = ''
        if type(action) is list: 
            prev_action_dest = action[-1] 
        else:
            prev_action_dest = str(action.args[-1])
            
        if self.verbose >= 1: print "prev_action_dest is now " + prev_action_dest 
        
        next_action_source = prev_action_dest
        next_action_dest = ''
        
        for atom in self.world.get_atoms(filters=["adjacent-"+dir,next_action_source]):
            if self.verbose >= 1: print "processing atom "+str(atom)
            if atom.args[0].name == next_action_source:
                next_action_dest = atom.args[1].name
                if self.verbose >= 1: print "next action dest = "+str(next_action_dest) 
                new_action_args = ['Curiosity',next_action_source,next_action_dest]
                subsequent_action = ['move'+dir]+new_action_args
                return subsequent_action
        
        return False
            
    def execute_action(self, action):
        '''
        Simulates the execution of an action and performs all updates
        to the world state and the agent's actions stored in memory.
        
        This includes not doing anything if the agent is stuck,
        as well as inserting 'stuck' and removing 'free' atoms
        if the agent ends up in quicksand.
        
        When an agent attempts to move in a direction, wind may push it farther in that direction.
        When wind pushes an agent, an agent will either be pushed an extra X number of tiles
        (where X is the wind strength). While being pushed, if any tile contains mud (aka quicksand)
        the agent will become stuck in that mud tile. (specifically the first mud tile encountered
        while being pushed) 
        
        '''
        # bump counter for actions executed
        self.mem.set(self.mem.ACTIONS_EXECUTED, 1+self.mem.get(self.mem.ACTIONS_EXECUTED))
    
        #print "dir(action) = "+str(dir(action))
        if 'move' in action.op:
            agent_at_atom = self.world.get_atoms(filters=['agent-at','Curiosity'])[0]
            agent_tile = agent_at_atom.args[1]
            #print "all quicksand atoms: "
            #for qs_atoms in self.world.get_atoms(filters=['quicksand']):
            #    print "  "+str(qs_atoms)
            #print "is_atom_true(quicksand, "+str([str(agent_tile)]) +") = "+str(self.world.is_true('quicksand',[str(agent_tile)]))
            if self.world.is_true('quicksand',[str(agent_tile)]):
                if self.verbose >= 1: print "free related atoms = "+str(map(str,self.world.get_atoms(filters=['free'])))
                if self.verbose >= 1: print 'self.world.is_true(free,Curiosity): '+str(self.world.is_true('free','Curiosity'))
                if self.world.is_true('free',['Curiosity']):
                    if self.verbose >= 1: print "Agent is free, moving away from quicksand"
                    # remove free
                    
                    # self.world.remove_fact('free',['Curiosity'])
                    # actually perform move action, assuming applicable
                    if self.world.midca_action_applicable(action):
                        if self.verbose >= 2:
                            print "simulating MIDCA action:", action
                        self.world.apply_midca_action(action)
                        if self.filehandle: self.filehandle.write("simulating MIDCA action:"+ str(action)+"\n")
                        return True
                    else:
                        if self.filehandle: self.filehandle.write("It seems that even though the agent is free, action "+str(action)+" is not applicable"+"\n")
                        print "It seems that even though the agent is free, action "+str(action)+" is not applicable"
                        return False
                else: # agent not free
                    if self.verbose >= 1: print "Agent is not free, failing to attempt to move from quicksand"
                    if self.filehandle: self.filehandle.write("Agent is not free, failing to attempt to move from quicksand"+"\n")
                    return False
                    # insert the first stuck atom if no stuck is already in the state
                    #stuck_atoms = self.world.get_atoms(filters=['stuck'])
                    #if len(stuck_atoms) == 0:
                    #    self.world.add_fact('stuck',['Curiosity'])
                        
            else: # no quicksand, just perform move like normal
                if self.verbose >= 2: print "action is "+str(action)
    
                if self.world.midca_action_applicable(action):
                    if self.verbose >= 2:
                        print "simulating MIDCA action:", action
                        
                        
                    # if move action, check to see which tiles the agent would pass in what order
                    # and check to see if those had quicksand
                    agent_stuck_inbetween = False
                    if self.verbose >=1 : print "len(action.args) = "+str(len(action.args))
                    if 'move' in str(action) and len(action.args) > 3:
                        #print "here"
                        start_tile = action.args[1]
                        tiles_passed = action.args[2:-1] # [0] = name (not a tile), [1] = start (already checked), [-1] = end (will be checked later)
                        #print "tiled passed = "+str(tiles_passed)
                        # check to see if any of these have mud, in order, and if they do
                        # place the agent there and make him stuck
                        for tile in tiles_passed:
                            if self.world.is_true('quicksand',[str(tile)]):
                                # remove free and agent's old location
                                self.world.remove_fact('free',['Curiosity'])
                                self.world.remove_fact('agent-at',['Curiosity', str(start_tile)])
                                # insert stuck and agents new location
                                self.world.add_fact('stuck',['Curiosity'])
                                self.world.add_fact('agent-at',['Curiosity', str(tile)])
                                agent_stuck_inbetween = True
                                print "Agent got stuck inbetween at loc "+str(tile)
                                if self.filehandle: self.filehandle.write("Agent got stuck inbetween at loc "+str(tile)+"\n")
                                break
                    if not agent_stuck_inbetween:
                        self.world.apply_midca_action(action)
                        if self.filehandle: self.filehandle.write("simulating MIDCA action:"+str(action)+"\n")
                        return True
                else:
                    if self.verbose >=1 : print "action "+str(action)+" is not applicable"
                    if self.filehandle: self.filehandle.write("action "+str(action)+" is not applicable"+"\n")
                    return False
                    
                    
        else: # an action other than move, no need to check if it will work
            if self.world.midca_action_applicable(action):
                if self.verbose >= 2:
                    print "simulating MIDCA action:", action
                self.world.apply_midca_action(action)
                if self.filehandle: self.filehandle.write("simulating MIDCA action:"+str(action)+"\n")
                return True
            else:
                if self.verbose >= 1: print "action "+str(action)+" is not applicable"
                if self.filehandle: self.filehandle.write("action "+str(action)+" is not applicable"+"\n")
        if self.filehandle: self.filehandle.write("did not execute action"+str(action)+" \n")
        return False
    
    def check_agent_in_mud(self):
        '''
        Returns true if agent stuck in mud, false otherwise.
        
        If agent IS stuck in mud, this function will update the state (i.e.
        remove the 'free' atom and add the 'stuck' atom into the state
        '''
        agent_stuck_in_mud = False
        agent_at_atom = self.world.get_atoms(filters=['agent-at','Curiosity'])[0]
        agent_tile = agent_at_atom.args[1]
        if self.world.is_true('quicksand',[str(agent_tile)]):
            stuck_atoms = self.world.get_atoms(filters=['stuck'])
            if len(stuck_atoms) == 0:
                self.world.add_fact('stuck',['Curiosity'])
                if self.verbose >= 1: print "inserted stuck atom"
                if self.filehandle: self.filehandle.write("inserted stuck atom"+"\n")
            if self.world.is_true('free',['Curiosity']):
                self.world.remove_fact('free',['Curiosity'])
                if self.verbose >= 1: print "removed free atom"
                if self.filehandle: self.filehandle.write("removed free atom"+"\n")
            agent_stuck_in_mud = True
            
        return agent_stuck_in_mud

    def sim_action_push(self):
        agent_at_atom = self.world.get_atoms(filters=['agent-at','Curiosity'])[0]
        # get agent's start loc
        if self.verbose >= 1: print "agent-at-atom = "+str(agent_at_atom)
        start = str(agent_at_atom[1])
        # get adjacent east atom
        adj_atoms_east_to_agent = self.world.get_atoms(filters=['adjacent-east',start])
        adj_atom_east = None
        for a in adj_atoms_east_to_agent:
            if self.verbose >= 1: print "looking at a = "+str(a)
            if str(a.args[0]) == start:
                adj_atom_east = a
        if self.verbose >= 1: print "adj_east_to_agent = "+str(adj_atom_east)
        # get new location
        dest = str(adj_atom_east.args[1])
        # now 'perform move'
        self.world.add_fact('agent-at',['Curiosity',dest])
        self.world.remove_fact('agent-at',['Curiosity',start])
        if self.verbose >= 1: print "Simulated a wind push from "+str(start)+" to "+str(dest)
        if self.filehandle: self.filehandle.write("Simulated a wind push from "+str(start)+" to "+str(dest)+"\n")
        
    def run(self, cycle, verbose = 2):
        '''
        nbeacons logic for obstacles:
        1. QUICKSAND: 
           i. If the agent executes a move command that results in the agent's new location
           being that same as a location of quicksand, then the simulator inserts 'stuck'
           and removes 'free' atoms from the state.
           ii. If the agent executes a move command and it is located in quicksand, if there is a
           'free' atom in the state, then the agent will successfully move out of the quicksand. 
           Otherwise it will remain in the same location and no change to the world state will occur.
           
        NOTE: current implementation only supports executing one action of a plan at a time, if
        multiple actions are selected by the agent and given to the simulator (by being stored in
        self.mem.ACTIONS) this will break
        '''
        self.mem.set(self.mem.MIDCA_CYCLES, 1+self.mem.get(self.mem.MIDCA_CYCLES))
        self.verbose = verbose
        if self.filehandle: self.filehandle.write("Cycle # "+str(cycle)+"\n")
        # update wind if on schedule
        for wind_sched_item in self.wind_schedule:
            if wind_sched_item[0] == cycle:
                self.wind_strength = wind_sched_item[1]
                print "Just changed wind strength to "+str(self.wind_strength)
                if self.filehandle: self.filehandle.write("Just changed wind strength to "+str(self.wind_strength)+"\n")
        
        first_action = None
        try:
            #get selected actions for this cycle. This is set in the act phase.
            actions = self.mem.get(self.mem.ACTIONS)[-1]
            first_action = actions[0]
        except TypeError, IndexError:
            if verbose >= 1:
                print "Simulator: no actions selected yet by MIDCA."
            if self.filehandle: self.filehandle.write("Simulator: no actions selected yet by MIDCA.\n")
            if self.filehandle: self.filehandle.write(nbeacons_util.drawNBeaconsScene(self.world,rtn_str=True)+"\n")
            if self.filehandle: self.filehandle.write("----------------------------------------------\n")
            if self.filehandle: self.filehandle.flush()
            return
        except:
            return

        # generate wind actions if applicable, and going in the same direction
        remaining_actions = [] # these will be wind pushes
        pushes_needed = 0
        
        # add subsequent actions depending on wind strength
        if self.wind and self.wind_dir in str(first_action):
            # check to see how far this move is
            start_x, start_y = map(int,str(first_action.args[1])[2:].split('y')) #hacky - relies on tiles represented as 'Tx2y3'
            end_x, end_y = map(int,str(first_action.args[-1])[2:].split('y')) # same kind of hackyness
            move_dist = (abs(end_x - start_x)+abs(end_y-start_y))
            if self.verbose >= 1: print "move_dist is "+str(move_dist)
            
            pushes_needed = self.wind_strength+1 - move_dist
            if self.verbose >= 1: print "  -->> pushes needed is "+str(pushes_needed)
            if self.filehandle: self.filehandle.write("  -->> pushes needed is "+str(pushes_needed)+"\n")

        # now start execution by executing the first action
        if not self.execute_action(first_action):
            if self.filehandle: self.filehandle.write(nbeacons_util.drawNBeaconsScene(self.world,rtn_str=True)+"\n")
            if self.filehandle: self.filehandle.write("----------------------------------------------\n")
            if self.filehandle: self.filehandle.flush()
            return

        if 'push' not in str(first_action):
            agent_stuck_in_mud = self.check_agent_in_mud()
        else:
            # we need to have a special case where, when the agent executes a push to become free
            # the agent doesn't immediately become stuck in mud again
            # so only when an action is not a 'push' will we check for, and insert, stuck atoms
            pass

        # now loop through the rest of the wind actions, unless the agent gets stuck in mud
        while pushes_needed > 0 and not agent_stuck_in_mud:
            # get next action
            #next_wind_action = remaining_actions[0]
            #remaining_actions = remaining_actions[1:]
            # execute action
            try:
                #print "About to apply named action "+str(next_wind_action)
                #self.world.apply_named_action(next_wind_action[0],next_wind_action[1:])
                self.sim_action_push()
                pushes_needed -= 1
                if self.verbose >= 1: print "Wind has blown the agent in the "+str(self.wind_dir)+" direction"
                if self.filehandle: self.filehandle.write("Wind has blown the agent in the "+str(self.wind_dir)+" direction\n")
            except:
                if self.verbose >= 1: print "Error executing sim push action : "+ str(sys.exc_info()[0])
                if self.filehandle: self.filehandle.write("Error executing sim push action : "+ str(sys.exc_info()[0])+"\n")
                break
            # check to see if agent in mud
            agent_stuck_in_mud = self.check_agent_in_mud()
        if self.filehandle: self.filehandle.write(nbeacons_util.drawNBeaconsScene(self.world,rtn_str=True)+"\n")
        if self.filehandle: self.filehandle.write("----------------------------------------------\n")
        if self.filehandle: self.filehandle.flush()

class CustomRunSimulator:
    '''
    This class is used to provide various kinds of commands to be used in customrun xml files (files that make it easier to run MIDCA experiments and control inputs, outputs, etc). For more information about using customrun config files, see customrun/customrun.py. Feel free to add any functions you need here.
    '''

    def writeDataToCSV(self,filename):
        ''' '''
        pass




