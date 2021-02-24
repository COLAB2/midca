## ===============================================================================
## Authors: AFRL/RQQA
## Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
## 
## Copyright (c) 2017 Government of the United State of America, as represented by
## the Secretary of the Air Force.  No copyright is claimed in the United States under
## Title 17, U.S. Code.  All Other Rights Reserved.
## ===============================================================================

## This file was auto-created by LmcpGen. Modifications will be overwritten.

class WaypointTransferMode:

    RequestWaypoints = 0
    AddWaypoints = 1
    ClearWaypoints = 2
    ReportWaypoints = 3



def get_WaypointTransferMode_str(str):
    """
    Returns a numerical value from a string
    """
    if str == "RequestWaypoints": return WaypointTransferMode.RequestWaypoints
    if str == "AddWaypoints": return WaypointTransferMode.AddWaypoints
    if str == "ClearWaypoints": return WaypointTransferMode.ClearWaypoints
    if str == "ReportWaypoints": return WaypointTransferMode.ReportWaypoints


def get_WaypointTransferMode_int(val):
    """
    Returns a string representation from an int
    """
    if val == WaypointTransferMode.RequestWaypoints: return "RequestWaypoints"
    if val == WaypointTransferMode.AddWaypoints: return "AddWaypoints"
    if val == WaypointTransferMode.ClearWaypoints: return "ClearWaypoints"
    if val == WaypointTransferMode.ReportWaypoints: return "ReportWaypoints"
    return WaypointTransferMode.RequestWaypoints


