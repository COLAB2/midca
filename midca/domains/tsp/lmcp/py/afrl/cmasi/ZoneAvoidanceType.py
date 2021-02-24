## ===============================================================================
## Authors: AFRL/RQQA
## Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
## 
## Copyright (c) 2017 Government of the United State of America, as represented by
## the Secretary of the Air Force.  No copyright is claimed in the United States under
## Title 17, U.S. Code.  All Other Rights Reserved.
## ===============================================================================

## This file was auto-created by LmcpGen. Modifications will be overwritten.

class ZoneAvoidanceType:

    Physical = 1
    Regulatory = 2
    Acoustic = 3
    Threat = 4
    Visual = 5



def get_ZoneAvoidanceType_str(str):
    """
    Returns a numerical value from a string
    """
    if str == "Physical": return ZoneAvoidanceType.Physical
    if str == "Regulatory": return ZoneAvoidanceType.Regulatory
    if str == "Acoustic": return ZoneAvoidanceType.Acoustic
    if str == "Threat": return ZoneAvoidanceType.Threat
    if str == "Visual": return ZoneAvoidanceType.Visual


def get_ZoneAvoidanceType_int(val):
    """
    Returns a string representation from an int
    """
    if val == ZoneAvoidanceType.Physical: return "Physical"
    if val == ZoneAvoidanceType.Regulatory: return "Regulatory"
    if val == ZoneAvoidanceType.Acoustic: return "Acoustic"
    if val == ZoneAvoidanceType.Threat: return "Threat"
    if val == ZoneAvoidanceType.Visual: return "Visual"
    return ZoneAvoidanceType.Physical


