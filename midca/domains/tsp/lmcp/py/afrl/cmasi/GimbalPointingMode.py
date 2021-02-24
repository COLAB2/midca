## ===============================================================================
## Authors: AFRL/RQQA
## Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
## 
## Copyright (c) 2017 Government of the United State of America, as represented by
## the Secretary of the Air Force.  No copyright is claimed in the United States under
## Title 17, U.S. Code.  All Other Rights Reserved.
## ===============================================================================

## This file was auto-created by LmcpGen. Modifications will be overwritten.

class GimbalPointingMode:

    Unknown = 0
    AirVehicleRelativeAngle = 1
    AirVehicleRelativeSlewRate = 2
    LatLonSlaved = 3
    InertialRelativeSlewRate = 4
    Scan = 5
    Stowed = 6



def get_GimbalPointingMode_str(str):
    """
    Returns a numerical value from a string
    """
    if str == "Unknown": return GimbalPointingMode.Unknown
    if str == "AirVehicleRelativeAngle": return GimbalPointingMode.AirVehicleRelativeAngle
    if str == "AirVehicleRelativeSlewRate": return GimbalPointingMode.AirVehicleRelativeSlewRate
    if str == "LatLonSlaved": return GimbalPointingMode.LatLonSlaved
    if str == "InertialRelativeSlewRate": return GimbalPointingMode.InertialRelativeSlewRate
    if str == "Scan": return GimbalPointingMode.Scan
    if str == "Stowed": return GimbalPointingMode.Stowed


def get_GimbalPointingMode_int(val):
    """
    Returns a string representation from an int
    """
    if val == GimbalPointingMode.Unknown: return "Unknown"
    if val == GimbalPointingMode.AirVehicleRelativeAngle: return "AirVehicleRelativeAngle"
    if val == GimbalPointingMode.AirVehicleRelativeSlewRate: return "AirVehicleRelativeSlewRate"
    if val == GimbalPointingMode.LatLonSlaved: return "LatLonSlaved"
    if val == GimbalPointingMode.InertialRelativeSlewRate: return "InertialRelativeSlewRate"
    if val == GimbalPointingMode.Scan: return "Scan"
    if val == GimbalPointingMode.Stowed: return "Stowed"
    return GimbalPointingMode.Unknown


