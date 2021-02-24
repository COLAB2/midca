## ===============================================================================
## Authors: AFRL/RQQA
## Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
## 
## Copyright (c) 2017 Government of the United State of America, as represented by
## the Secretary of the Air Force.  No copyright is claimed in the United States under
## Title 17, U.S. Code.  All Other Rights Reserved.
## ===============================================================================

## This file was auto-created by LmcpGen. Modifications will be overwritten.

class TravelMode:

    SinglePass = 0
    ReverseCourse = 1
    Loop = 2



def get_TravelMode_str(str):
    """
    Returns a numerical value from a string
    """
    if str == "SinglePass": return TravelMode.SinglePass
    if str == "ReverseCourse": return TravelMode.ReverseCourse
    if str == "Loop": return TravelMode.Loop


def get_TravelMode_int(val):
    """
    Returns a string representation from an int
    """
    if val == TravelMode.SinglePass: return "SinglePass"
    if val == TravelMode.ReverseCourse: return "ReverseCourse"
    if val == TravelMode.Loop: return "Loop"
    return TravelMode.SinglePass


