# This file contains helpful functions for the mars world 

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
                # x is greater, therefore dest is west
                if self.westTile:
                    raise Exception("Error: trying to set westTile to "+str(dest)+" but westTile already set to "+str(self.westTile))
                self.westTile = dest
            else:
                # x is less, therefore dest is east
                if self.eastTile:
                    raise Exception("Error: trying to set eastTile to "+str(dest)+" but eastTile already set to "+str(self.eastTile))
                self.eastTile = dest
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
        str = ""
        if self.northTile:
            str += "adjacent-north("+str(self)+","+str(self.northTile)+")\n"
        if self.southTile:
            str += "adjacent-south("+str(self)+","+str(self.southTile)+")\n"
        if self.eastTile:
            str += "adjacent-east("+str(self)+","+str(self.eastTile)+")\n"
        if self.westTile:
            str += "adjacent-west("+str(self)+","+str(self.westTile)+")\n"
        return str
    
def generate_tiles(width=10,height=10):
    
    
def makeasciiframe(self, state):
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

def generate_strips_domain_str():
    # we want the agent in an X by Y domain size of tiles
    X = 3 # width
    Y = 3 # height
    
    str = ""
    tile_strs = []
    #1: make all the tiles
    for i in range(X):
        for j in range(Y):
            t_str = "Tx"+i+"y"+j 
            str += "TILE("+t_str+")\n"
    
    #2: make tiles properly adjacent
    
