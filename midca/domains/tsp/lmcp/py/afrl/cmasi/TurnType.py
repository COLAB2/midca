## ===============================================================================
## Authors: AFRL/RQQA
## Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
## 
## Copyright (c) 2017 Government of the United State of America, as represented by
## the Secretary of the Air Force.  No copyright is claimed in the United States under
## Title 17, U.S. Code.  All Other Rights Reserved.
## ===============================================================================

## This file was auto-created by LmcpGen. Modifications will be overwritten.

class TurnType:

    TurnShort = 0
    FlyOver = 1



def get_TurnType_str(str):
    """
    Returns a numerical value from a string
    """
    if str == "TurnShort": return TurnType.TurnShort
    if str == "FlyOver": return TurnType.FlyOver


def get_TurnType_int(val):
    """
    Returns a string representation from an int
    """
    if val == TurnType.TurnShort: return "TurnShort"
    if val == TurnType.FlyOver: return "FlyOver"
    return TurnType.TurnShort


