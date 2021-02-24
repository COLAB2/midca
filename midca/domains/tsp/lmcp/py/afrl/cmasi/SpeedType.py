## ===============================================================================
## Authors: AFRL/RQQA
## Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
## 
## Copyright (c) 2017 Government of the United State of America, as represented by
## the Secretary of the Air Force.  No copyright is claimed in the United States under
## Title 17, U.S. Code.  All Other Rights Reserved.
## ===============================================================================

## This file was auto-created by LmcpGen. Modifications will be overwritten.

class SpeedType:

    Airspeed = 0
    Groundspeed = 1



def get_SpeedType_str(str):
    """
    Returns a numerical value from a string
    """
    if str == "Airspeed": return SpeedType.Airspeed
    if str == "Groundspeed": return SpeedType.Groundspeed


def get_SpeedType_int(val):
    """
    Returns a string representation from an int
    """
    if val == SpeedType.Airspeed: return "Airspeed"
    if val == SpeedType.Groundspeed: return "Groundspeed"
    return SpeedType.Airspeed


