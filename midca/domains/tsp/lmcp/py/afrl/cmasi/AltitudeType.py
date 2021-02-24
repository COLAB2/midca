## ===============================================================================
## Authors: AFRL/RQQA
## Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
## 
## Copyright (c) 2017 Government of the United State of America, as represented by
## the Secretary of the Air Force.  No copyright is claimed in the United States under
## Title 17, U.S. Code.  All Other Rights Reserved.
## ===============================================================================

## This file was auto-created by LmcpGen. Modifications will be overwritten.

class AltitudeType:

    AGL = 0
    MSL = 1



def get_AltitudeType_str(str):
    """
    Returns a numerical value from a string
    """
    if str == "AGL": return AltitudeType.AGL
    if str == "MSL": return AltitudeType.MSL


def get_AltitudeType_int(val):
    """
    Returns a string representation from an int
    """
    if val == AltitudeType.AGL: return "AGL"
    if val == AltitudeType.MSL: return "MSL"
    return AltitudeType.AGL


