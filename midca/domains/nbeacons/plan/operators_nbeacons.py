"""
NBeacons operators for PyHOP
"""
from midca.modules._plan import pyhop

""" World coordinates are bounded by [0,state.dim-1] for both x and y
"""

""" move down """
# start and dest are only to help conversion to MIDCA operators, they are not used in 
# PyHop Planning
def movesouth(state, agent, start, dest):
    if state.agents[agent] not in state.mud.values():
        xy_str = state.agents[agent]
        x = int(xy_str.split(',')[0])
        y = int(xy_str.split(',')[1])
        new_y = y
        if y == 0:
            return False
        else:
            new_y = y - 1

        new_xy_str = str(x)+','+str(new_y)
        state.agents[agent] = new_xy_str
        return state
    else:
        return False

""" move up """
def movenorth(state, agent, start, dest):
    if state.agents[agent] not in state.mud.values():
        xy_str = state.agents[agent]
        x = int(xy_str.split(',')[0])
        y = int(xy_str.split(',')[1])
        new_y = y
        if y == state.dim['dim']-1:
            return False
        else:
            new_y = y + 1

        new_xy_str = str(x)+','+str(new_y)
        state.agents[agent] = new_xy_str
        return state
    else:
        return False

""" move left """
def movewest(state, agent, start, dest):
    if state.agents[agent] not in state.mud.values():
        xy_str = state.agents[agent]
        x = int(xy_str.split(',')[0])
        y = int(xy_str.split(',')[1])
        new_x = x
        if x == 0:
            return False
        else:
            new_x = x - 1

        new_xy_str = str(new_x)+','+str(y)
        state.agents[agent] = new_xy_str
        return state
    else:
        return False

""" move right """
def moveeast(state, agent, start, dest):
    if state.agents[agent] not in state.mud.values():
        xy_str = state.agents[agent]
        x = int(xy_str.split(',')[0])
        y = int(xy_str.split(',')[1])
        new_x = x
        if x == state.dim['dim']-1:
            return False
        else:
            new_x = x + 1

        new_xy_str = str(new_x)+','+str(y)
        state.agents[agent] = new_xy_str
        return state
    else:
        return False

def activatebeacon(state, agent, loc, bcn_id):
    agent_at_beacon = False
    beacon_key = None
    for beaconstr, beaconlocstr in state.beaconlocs.items():
        if state.agents[agent] == beaconlocstr:
            #print("     [" + beaconstr + "] = "+beaconlocstr + " (and state.agents[agent] = "+state.agents[agent])
            agent_at_beacon = True
            beacon_key = beaconstr
            break
    
    #print("beaconstr = "+str(beaconstr)+", state.activated.keys() = "+str(state.activated.keys())+" agent at beacon = "+str(agent_at_beacon))
    beacon_exists = beacon_key in state.activated.keys() 
    if agent_at_beacon:
        if beacon_exists and state.activated[beacon_key]:
            return False # already activated
        else:
            state.activated[beacon_key] = True
            return state
    else:
        return False   

# from IJCAI-15 work
# def activatebeacon(state, agent):
#     if state.agents[agent]:
#         xy_str = state.agents[agent]
#         x = int(xy_str.split(',')[0])
#         y = int(xy_str.split(',')[1])
#         new_x = x
#         print("in activate beacon with xy_str="+str(xy_str)+" and beaconlocs = "+str(state.beacons.values()))
#         if xy_str in state.beacons.values():
#             # get the beacon_id at this loc
#             b_id = [b for (b,loc) in state.beaconlocs.items() if loc == xy_str][0]
#             state.activated[b_id] = True
#             return state
#         else:
#             return False # there is no beacon here
#     else:
#         return False    
    
def declare_operators():
    pyhop.declare_operators(movenorth, movesouth, moveeast, movewest, activatebeacon)



# def loadtruck(state, obj, truck):
#     if state.trucks[truck] == state.pkgs[obj]:
#         state.pkgs[obj] = truck
#         return state
#     else: return False

# def unloadtruck(state, obj, truck):
#     if state.pkgs[obj] == truck:
#         state.pkgs[obj] = state.trucks[truck]
#         return state
#     else: return False    

# def loadairplane(state, obj, apln):
#     if state.pkgs[obj] == state.aplns[apln]:
#         state.pkgs[obj] = apln
#         return state
#     else: return False

# def unloadairplane(state, obj, apln):
#     if state.pkgs[obj] == apln:
#         state.pkgs[obj] = state.aplns[apln]
#         return state
#     else: return False    

# def drivetruck(state, truck, locfrom, locto):
#     if (state.trucks[truck] == locfrom and
#             state.city[state.trucks[truck]] == state.city[locto]): # trucks can only travel within a city
#         state.trucks[truck] = locto
#         return state
#     else: return False

# def flyairplane(state, apln, locfrom, locto):
#     if state.aplns[apln] == locfrom:
#         state.aplns[apln] = locto
#         return state
#     else: return False

# pyhop.declare_operators(loadtruck, unloadtruck, loadairplane, unloadairplane, drivetruck, flyairplane)
