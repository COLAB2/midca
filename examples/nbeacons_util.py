# This file contains helpful functions for the nbeacons domain

import random

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
            if xdiff > 0:
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
            
            
def makeasciiframe(self, state): # state is a pyhop state
    '''
    Given a pyhop state, returns a drawing of the agent's state in ascii
    '''
    DIRT = '.'
    BEACON_UNACTIVATED = 'b'
    BEACON_ACTIVATED = 'B'                        
    WOOD = 'x'
    WOOD_ON_FIRE = '*'
    AGENT = 'a'
    AGENT_WITH_UNACTIVATED_BEACON = '8'
    AGENT_WITH_ACTIVATED_BEACON = 'A'
    AGENT_WITH_WOOD = '^'
    AGENT_WITH_FIRE = '%'
    AGENT_WITH_FLARE = '#'
    FLARE = '$'


    # initialize the map with dirt
    dim = state.dim
    grid = []
    for c in range(dim):
        row = []
        for r in range(dim):
                row.append(DIRT)
        grid.append(row)
    
    # Add agent
    agentstr = state.agents[config.settings['AGENT_NAME']]
    agent_x = int(agentstr.split(',')[0]) 
    agent_y = int(agentstr.split(',')[1])
    grid[agent_y][agent_x] = AGENT
    
    # now go through each attribute of the state, changing the grid
    # ADD ALL BEACONS
    for beaconkey in state.activated.keys():
        beaconstr = beaconkey[1:] # remove the 'b' at the beginning
        x = int(beaconstr.split(",")[0]) 
        y = int(beaconstr.split(",")[1]) 
        
        if state.activated[beaconkey]:
            if state.beaconlocs[beaconkey] == state.agents[config.AGENT_NAME]:
                # Beacon is activated, with agent
                grid[y][x] = AGENT_WITH_ACTIVATED_BEACON
            else:
                # Beacon is activated, no agent
                grid[y][x] = BEACON_ACTIVATED
        else:
            if state.beaconlocs[beaconkey] == state.agents[config.settings['AGENT_NAME']]:
                # Beacon is activated, with agent
                grid[y][x] = AGENT_WITH_UNACTIVATED_BEACON
            else:
                # Beacon is unactivated, no agent
                grid[y][x] = BEACON_UNACTIVATED 
                
    if state.domainname == 'MARSWORLD':
        for woodkey in state.wood.keys():
            woodstr = woodkey[1:] # remove the 'w' at the beginning
            x = int(woodstr.split(",")[0]) 
            y = int(woodstr.split(",")[1]) 
            
            if state.fires[woodstr]:
                if woodstr == state.agents[config.AGENT_NAME]:
                    # There is a fire with the agent
                    grid[y][x] = AGENT_WITH_FIRE
                else:
                    # fire no agent
                    grid[y][x] = WOOD_ON_FIRE
            else:
                if state.wood[woodkey] == state.agents[config.settings['AGENT_NAME']]:
                    # wood with agent
                    grid[y][x] = AGENT_WITH_WOOD
                else:
                    # wood no agent
                    grid[y][x] = WOOD 
        
         
        for flare_id,flareloc in state.flarelocs.items():
            if ',' in flareloc: #crude test to see if flareloc is an xystr
                x = int(flareloc.split(',')[0])
                y = int(flareloc.split(',')[1])
                if state.flareslit[flare_id]:
                    if flareloc == state.agents[config.settings['AGENT_NAME']]:
                        # There is a lit flare with the agent
                        grid[y][x] = AGENT_WITH_FLARE
                    else:
                        # flare no agent
                        grid[y][x] = FLARE

    return grid
    
if __name__ == "__main__":
    env1 = NBeaconGrid()
    env1.generate()
    print(env1.get_STRIPS_str())
