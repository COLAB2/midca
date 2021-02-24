## ===============================================================================
## Authors: AFRL/RQQA
## Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
## 
## Copyright (c) 2017 Government of the United State of America, as represented by
## the Secretary of the Air Force.  No copyright is claimed in the United States under
## Title 17, U.S. Code.  All Other Rights Reserved.
## ===============================================================================

## This file was auto-created by LmcpGen. Modifications will be overwritten.

class LoiterDirection:

    VehicleDefault = 0
    CounterClockwise = 1
    Clockwise = 2



def get_LoiterDirection_str(str):
    """
    Returns a numerical value from a string
    """
    if str == "VehicleDefault": return LoiterDirection.VehicleDefault
    if str == "CounterClockwise": return LoiterDirection.CounterClockwise
    if str == "Clockwise": return LoiterDirection.Clockwise


def get_LoiterDirection_int(val):
    """
    Returns a string representation from an int
    """
    if val == LoiterDirection.VehicleDefault: return "VehicleDefault"
    if val == LoiterDirection.CounterClockwise: return "CounterClockwise"
    if val == LoiterDirection.Clockwise: return "Clockwise"
    return LoiterDirection.VehicleDefault


