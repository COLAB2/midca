from midca.domains.tsp.lmcp.py.lmcp.LMCPObject import *

## ===============================================================================
## Authors: AFRL/RQQA
## Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
## 
## Copyright (c) 2017 Government of the United State of America, as represented by
## the Secretary of the Air Force.  No copyright is claimed in the United States under
## Title 17, U.S. Code.  All Other Rights Reserved.
## ===============================================================================

## This file was auto-created by LmcpGen. Modifications will be overwritten.

from midca.domains.tsp.lmcp.py.afrl.vehicles import GroundVehicleConfiguration
from midca.domains.tsp.lmcp.py.afrl.vehicles import GroundVehicleState
from midca.domains.tsp.lmcp.py.afrl.vehicles import SurfaceVehicleConfiguration
from midca.domains.tsp.lmcp.py.afrl.vehicles import SurfaceVehicleState
from midca.domains.tsp.lmcp.py.afrl.vehicles import StationarySensorConfiguration
from midca.domains.tsp.lmcp.py.afrl.vehicles import StationarySensorState


SERIES_NAME = "VEHICLES"
#Series Name turned into a long for quick comparisons.
SERIES_NAME_ID = 6216454340153722195
SERIES_VERSION = 1


class SeriesEnum:

    def getName(self, type_):
        if(type_ ==  1): return "GroundVehicleConfiguration"
        if(type_ ==  2): return "GroundVehicleState"
        if(type_ ==  3): return "SurfaceVehicleConfiguration"
        if(type_ ==  4): return "SurfaceVehicleState"
        if(type_ ==  5): return "StationarySensorConfiguration"
        if(type_ ==  6): return "StationarySensorState"


    def getType(self, name):
        if ( name == "GroundVehicleConfiguration"): return 1
        if ( name == "GroundVehicleState"): return 2
        if ( name == "SurfaceVehicleConfiguration"): return 3
        if ( name == "SurfaceVehicleState"): return 4
        if ( name == "StationarySensorConfiguration"): return 5
        if ( name == "StationarySensorState"): return 6

        return -1

    def getInstance(self, type_):
        if(type_ ==  1): return GroundVehicleConfiguration.GroundVehicleConfiguration()
        if(type_ ==  2): return GroundVehicleState.GroundVehicleState()
        if(type_ ==  3): return SurfaceVehicleConfiguration.SurfaceVehicleConfiguration()
        if(type_ ==  4): return SurfaceVehicleState.SurfaceVehicleState()
        if(type_ ==  5): return StationarySensorConfiguration.StationarySensorConfiguration()
        if(type_ ==  6): return StationarySensorState.StationarySensorState()

        return None
