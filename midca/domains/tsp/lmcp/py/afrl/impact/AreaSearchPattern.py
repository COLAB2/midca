## ===============================================================================
## Authors: AFRL/RQQA
## Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
## 
## Copyright (c) 2017 Government of the United State of America, as represented by
## the Secretary of the Air Force.  No copyright is claimed in the United States under
## Title 17, U.S. Code.  All Other Rights Reserved.
## ===============================================================================

## This file was auto-created by LmcpGen. Modifications will be overwritten.

class AreaSearchPattern:

    Spiral = 0
    Sector = 1
    Sweep = 2



def get_AreaSearchPattern_str(str):
    """
    Returns a numerical value from a string
    """
    if str == "Spiral": return AreaSearchPattern.Spiral
    if str == "Sector": return AreaSearchPattern.Sector
    if str == "Sweep": return AreaSearchPattern.Sweep


def get_AreaSearchPattern_int(val):
    """
    Returns a string representation from an int
    """
    if val == AreaSearchPattern.Spiral: return "Spiral"
    if val == AreaSearchPattern.Sector: return "Sector"
    if val == AreaSearchPattern.Sweep: return "Sweep"
    return AreaSearchPattern.Spiral


