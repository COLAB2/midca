# This file contains helpful functions for the nbeacons domain

import random
import MIDCA.modules.planning
from MIDCA.modules._plan import pyhop

class NBeaconGrid():
    '''
    Class representing a single instantion of an NBeacon's grid, which includes:
    - Tiles
    - Beacons
    - Agent
    '''

    TILE_GRID = []
    BEACONS = []
    DIM = -1
    AGENT_NAME = 'Curiosity'
    
    def __init__(self):
        pass
    
    def generate(self,width=10,height=10,num_beacons=10):
        if width != height:
            raise Exception("Sorry but only square environments are valid: width must equal height")
        self.DIM = width
        self.TILE_GRID = self.generate_tiles(width,height)
        self.BEACONS = self.generate_beacons(num_beacons)
    
    def generate_tiles(self,width=10,height=10):
        '''
        Given the following parameters, returns a list of tiles:
        width (int)
        height (int)
    
        TODO: make the function faster if we want to generate big states (like 100 x 100 or more)
        '''
        TILE_GRID = []
        for w in range(width):
            TILE_COLUMN = []
            for h in range(height):
                TILE_COLUMN.append(Tile(w,h))
            TILE_GRID.append(TILE_COLUMN)
                
        # go through all tiles and set adjacent to
        adj_counts = 0
        for w1 in range(width):
            for h1 in range(height):
                curr_tile = TILE_GRID[w1][h1]
                for w2 in range(width):
                    for h2 in range(height):
                        try:
                            if curr_tile.set_adjacent_to(TILE_GRID[w2][h2]):
                                #print(str(adj_counts)+" set "+str(curr_tile)+" adj to "+str(TILE_GRID[w2][h2]))
                                adj_counts+=1
                        except:
                            # do nothing
                            pass
        return TILE_GRID
    
    def generate_beacons(self, num_beacons=10):
        '''
          Generates the given number of beacons. For each beacon the following is generated:
          - id (i..e B1, B2, etc)
          - tile (where the beacon is located)
          - activated (defaults to False)
          - type (defaults to 1)
        '''
        
        BEACONS = []
        tiles_chosen = []
        
        for i in range(num_beacons):
            # get a random tile
            ran_tile = random.choice(random.choice(self.TILE_GRID))
            while ran_tile in tiles_chosen:
                # loop until we get a unique tile
                ran_tile = random.choice(random.choice(self.TILE_GRID))
            tiles_chosen.append(ran_tile)
            BEACONS.append(Beacon(i,ran_tile)) # add type later
            
        return BEACONS
    
    def get_STRIPS_str(self):
        '''
        Returns the grid in a STRIPS format, to be loaded by MIDCA
        
        NOTE: the order in which the lines are outputted is important (i.e.
              all the type declarations have to happen before predicates)
        '''    
    
        if len(self.TILE_GRID) <= 0:
            raise Exception("Attempting to STRIPS str on empty TILE_GRID. Did you call generate() first?")
    
        strips_result_str = ""
    
        # add dimension
        strips_result_str += "DIM("+str(self.DIM)+")\n"
        
        # First, place all the tiles
        for t_row in self.TILE_GRID:
            for t in t_row:
                strips_result_str += t.get_tile_STRIPS_str()
    
        # Second, do all the beacons
        for b in self.BEACONS:
            strips_result_str += b.get_STRIPS_str()

        # get all the adjacency strings
        for t_row in self.TILE_GRID:
            for t in t_row:
                strips_result_str += t.get_all_adjacent_STRIPS_str()
                
        # get all strings related to beacons
        for b in self.BEACONS:
            strips_result_str += b.get_all_STRIPS_str()

        # add agent's location
        x = random.choice(range(self.DIM))
        y = random.choice(range(self.DIM))
        strips_result_str += "AGENT("+str(self.AGENT_NAME)+")\n"
        strips_result_str += "agent-at("+str(self.AGENT_NAME)+","+str("Tx"+str(x)+"y"+str(y))+")\n"
        
        
        return strips_result_str

class Beacon():
    
    id = -1
    tile = None
    type = None # int representing type
    activated = None # boolean
    
    def __init__(self, id, tile, type=1, activated=False,):
        self.id = id
        self.tile = tile
        self.type = type
        self.activated = activated
        
    def get_STRIPS_str(self):
        s = ""
        s += "BEACON(B"+str(self.id)+")\n"
        return s
    
    def get_all_STRIPS_str(self):
        s = ""
        s += "beacon-at(B"+str(self.id)+","+str(self.tile)+")\n"
        return s
    
class Tile():
    X = -1
    Y = -1
    northTile = None
    southTile = None
    eastTile = None
    westTile = None
    
    def __init__(self,x,y):
        self.X = x
        self.Y = y
    
    def getX(self):
        return self.X
    
    def getY(self):
        return self.Y
    
    def getNorthTile(self):
        return self.northTile
    
    def getSouthTile(self):
        return self.southTile
    
    def getEastTile(self):
        return self.eastTile
    
    def getWestTile(self):
        return self.westTile
    
    def set_adjacent_to(self,dest):
        '''
        takes a dest of type Tile

        checks/calculates the following:
        1) tile coordinates are valid
        2) tile is actually adjacent
        3) what direction the given tile is from our tile
        4) that we don't already have a tile in that direction
        
        Returns True if successfully set adjacent to, otherwise raises
         an exception with a helpful error message
        
        north means its y is less than ours
        south means its y is greater than ours
        east means its x is less than ours
        west means its x is greater than ours
        ''' 
        
        # 1) check tile coordinates are within the grid
        if dest.getX() < 0 or dest.getY() < 0:
            raise Exception("Tile "+str(dest)+" has invalid coordinates: x "+dest.getX()+ " and y "+dest.getY())
        
        # 2) Check that tile is actually adjacent
        #    first get the x and y differences
        xdiff = self.X - dest.getX()
        ydiff = self.Y - dest.getY()
        
        if ((xdiff != 0 and ydiff != 0) or
            (xdiff == 0 and ydiff == 0) or
            (xdiff == 0 and abs(ydiff) != 1) or
            (abs(xdiff) != 1 and ydiff == 0)):
            raise Exception("Tile "+str(dest)+" is not adjacent, xdiff is "+str(xdiff)+", ydiff is "+str(ydiff))
        
        # 3 & 4) Figure out what direction the tile is, and check not already set
        if xdiff != 0:
            if xdiff < 0:
                # x is less than (b/c diff), therefore dest is east
                if self.eastTile:
                    raise Exception("Error: trying to set eastTile to "+str(dest)+" but eastTile already set to "+str(self.eastTile))
                self.eastTile = dest
                
            else:
                # x is less, therefore dest is west
                if self.westTile:
                    raise Exception("Error: trying to set westTile to "+str(dest)+" but westTile already set to "+str(self.westTile))
                self.westTile = dest
        else:
            if ydiff > 0:
                # y is greater, therefore dest is south
                if self.southTile:
                    raise Exception("Error: trying to set southTile to "+str(dest)+" but southTile already set to "+str(self.southTile))
                self.southTile = dest
            else:
                # y is less, therefore dest is north
                if self.northTile:
                    raise Exception("Error: trying to set northTile to "+str(dest)+" but northTile already set to "+str(self.northTile))
                self.northTile = dest
        
        return True
        
        
    def __str__(self):
        # Simple str for identifying this tile
        return "Tx"+str(self.X)+"y"+str(self.Y)
    
    def get_tile_STRIPS_str(self):
        return "TILE("+str(self)+")\n"
    
    def get_all_adjacent_STRIPS_str(self):
        '''
        Returns all STRIPS strings for loading the state in MIDCA
        '''
        result_str = ""
        if self.northTile:
            result_str += "adjacent-north("+str(self)+","+str(self.northTile)+")\n"
        if self.southTile:
            result_str += "adjacent-south("+str(self)+","+str(self.southTile)+")\n"
        if self.eastTile:
            result_str += "adjacent-east("+str(self)+","+str(self.eastTile)+")\n"
        if self.westTile:
            result_str += "adjacent-west("+str(self)+","+str(self.westTile)+")\n"
        return result_str
    
    def __eq__(self,anotherTile):
        return self.X == anotherTile.getX() and self.Y == anotherTile.getY()
            
def convert(midca_tile_str):
    '''
    Converts a MIDCA tile str like Tx4y6 into
    a PyHOP str 4,6
    
    (it's not that PyHOP in general uses 4,6, just what the 
    operators are expecting) 
    '''
    if midca_tile_str.startswith("Tx"):
        # replace y with a comma
        new_v = midca_tile_str.replace("y",",")
        #trim off Tx at the beginning
        new_v = new_v[2:]
        print("new_v is "+str(new_v))
        return new_v
    else:
        print("hit the else")
        return midca_tile_str

def pyhop_state_from_world(world, name = "state"):
    s = pyhop.State(name)
    s.agents={'curiosity':'3,3'} # put beacons here too? and mud tiles?
    s.dim={'dim':-1}
    s.agents = {}
    s.beaconlocs = {} # key is beacon id (e.g. b1), val is tile str like Tx3y4 
    s.beacontypes = {} # key is beacon id (e.g. b1), val is a number representing the type
    s.activated = {} # key is beacon id (e.g. b1), val is True if activated, False otherwise
    s.agents = {}
    s.mud = {}
    beacons = []
    agent = None
    for objname in world.objects:
        if world.objects[objname].type.name == "BEACON":
            beacons.append(objname)
        elif world.objects[objname].type.name == "AGENT" and not agent: # if agent already set, means multi-agent
            agent = objname
        elif world.objects[objname].type.name == "DIM":
            s.dim['dim'] = int(objname)
            
    # by default, make all beacons deactivated (if they are activated, will be changed below)        
    for bcn in beacons:
        s.activated[bcn] = False
    
    # now process atoms
    for atom in world.atoms:
        if atom.predicate.name == "beacon-at":
            b_id = atom.args[0].name
            tile_str = atom.args[1].name
            s.beaconlocs[b_id] = tile_str
        elif atom.predicate.name == "activated":
            b_id = atom.args[0].name
            s.activated[b_id] = True
        elif atom.predicate.name == "agent-at":
            s.agents[atom.args[0].name] = convert(atom.args[1].name) 
            
    # convert tile names to pyhop operators
    for (k,v) in s.beaconlocs.items():
        s.beaconlocs[k] = convert(v)
        
    
            
    #print("at the end of nbeacons_pyhop_state_from_world:")
    #print_state(s)
    return s

#note: str(arg) must evaluate to the name of the arg in the world representation for this method to work.
def pyhop_tasks_from_goals(goals,pyhopState):
    alltasks = []
    beacongoals = pyhop.Goal("goals")
    beacongoals.activated = {}
    perimeter_goal_locs = []
    agent_at_goal_locs = []
    agent_name = ""
    
    print("goals = "+str(goals))
    for goal in goals:
        #extract predicate
        if 'predicate' in goal.kwargs:
            predicate = str(goal.kwargs['predicate'])
        elif 'Predicate' in goal.kwargs:
            predicate = str(goal.kwargs['Predicate'])
        elif goal.args:
            predicate = str(goal.args[0])
        else:
            raise ValueError("Goal " + str(goal) + " does not translate to a valid pyhop task")
        args = [str(arg) for arg in goal.args]
        print("args[0] = "+str(args[0]))
        print("predicate = "+str(predicate))
        if args[0] == predicate:
            args.pop(0)
        if predicate == "activated":
            loc = pyhopState.beaconlocs[str(args[0])]
            perimeter_goal_locs.append(loc)
        if predicate == "agent-at":
            print("here")
            agent_dest = convert(args[0])
            agent_at_goal_locs.append(agent_dest)
            print("annndd here")

    # important, only one goal for all activated beacons
    if perimeter_goal_locs:
        alltasks.append(("make_perimeter",pyhopState.agents.keys()[0],perimeter_goal_locs))

    print("agent_at_goal_locs: "+str(agent_at_goal_locs))        
    if agent_at_goal_locs:
        print("adding navigate task:"+str(("navigate",pyhopState.agents.keys()[0],agent_at_goal_locs)))
        alltasks.append(("navigate",pyhopState.agents.keys()[0],agent_at_goal_locs))
    
    return alltasks


def nbeacons_plans_pyhop_to_midca(plan,state):
    '''
    Returns a plan that can be executed by MIDCA
    '''
    print("about to print the state")
    midcaPlan = []
    agents_xy_str = state.agents['Curiosity']
    midca_xy_str = "Tx" + agents_xy_str.replace(",","y")
    for a in plan:
        print("a="+str(a))
        print("a[0]="+str(a[0]))
        
        if a[0] == 'moveeast':
            new_a = (a[0],a[1],)
        elif a[0] == 'movewest':
            pass
        elif a[0] == 'movenorth':
            pass
        elif a[0] == 'movesouth':
            pass
    #pyhop.print_state(state)
    

def asciiframestr(frame, numbered_borders = True):
    print("in asciiframestr")
    result = ""
    if numbered_borders:
        result +="  "
        for i in range(len(frame)):
            result += str(i)+" "
        result+='\n'
    r = 0
    for row in frame:
        if numbered_borders:
            result += str(r)+" "
        for val in row:
            result += val + " "
        result += "\n"
        r+=1
    # remove trailing newline
    result = result[:len(result)-2]
    return result   
   
def drawNBeaconsScene(midcastate):
    '''
    Takes the world state MIDCA and returns a str of an ascii
    drawing of the nbeacons domain for visual consumption by a human
    '''
    
    # SYMBOLS
    DIRT = '.'
    BEACON_UNACTIVATED = 'b'
    BEACON_ACTIVATED = 'B'                        
    WOOD = 'x'
    #WOOD_ON_FIRE = '*'
    AGENT = 'a'
    AGENT_WITH_UNACTIVATED_BEACON = '*'
    AGENT_WITH_ACTIVATED_BEACON = 'A'
    AGENT_WITH_WOOD = '^'
    AGENT_WITH_FIRE = '%'
    AGENT_WITH_FLARE = '#'
    FLARE = '$'
    
    # convert MIDCA world to PyHop State (only doing this for code re-use
    pyhopState = pyhop_state_from_world(midcastate)
    print("successfully created pyhop state from midca state")
    # initialize the map with dirt
    dim = pyhopState.dim['dim']
    grid = []
    for c in range(dim):
        row = []
        for r in range(dim):
                row.append(DIRT)
        grid.append(row)
    print("here2")
    # Add agent
    agentstr = pyhopState.agents['Curiosity']
    agent_x = int(agentstr.split(',')[0]) 
    agent_y = int(agentstr.split(',')[1])
    grid[agent_y][agent_x] = AGENT
     
    # now go through each attribute of the state, changing the grid
    # ADD ALL BEACONS
    for beaconkey in pyhopState.activated.keys():
        beaconstr = pyhopState.beaconlocs[beaconkey]
        x = int(beaconstr.split(",")[0]) 
        y = int(beaconstr.split(",")[1]) 
        
        if pyhopState.activated[beaconkey]:
            if pyhopState.beaconlocs[beaconkey] == pyhopState.agents['Curiosity']:
                # Beacon is activated, with agent
                grid[y][x] = AGENT_WITH_ACTIVATED_BEACON
            else:
                # Beacon is activated, no agent
                grid[y][x] = BEACON_ACTIVATED
        else:
            if pyhopState.beaconlocs[beaconkey] == pyhopState.agents['Curiosity']:
                # Beacon is activated, with agent
                grid[y][x] = AGENT_WITH_UNACTIVATED_BEACON
            else:
                # Beacon is unactivated, no agent
                beacon_id = ""
                #pyhop.print_state(pyhopState)
                for (k,v) in pyhopState.beaconlocs.items():
                    if v == str(x)+","+str(y):
                        beacon_id = k[1:] # remove the 'B'
                
                #grid[y][x] = BEACON_UNACTIVATED 
                grid[y][x] = beacon_id
 
    print("successfully computed grid for drawing")
    print(asciiframestr(grid))


    
if __name__ == "__main__":
    env1 = NBeaconGrid()
    env1.generate()
    print(env1.get_STRIPS_str())
