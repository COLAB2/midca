## ===============================================================================
## Authors: AFRL/RQQA
## Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
## 
## Copyright (c) 2017 Government of the United State of America, as represented by
## the Secretary of the Air Force.  No copyright is claimed in the United States under
## Title 17, U.S. Code.  All Other Rights Reserved.
## ===============================================================================

## This file was auto-created by LmcpGen. Modifications will be overwritten.

class NavigationMode:

    Waypoint = 0
    Loiter = 1
    FlightDirector = 2
    TargetTrack = 3
    FollowLeader = 4
    LostComm = 5



def get_NavigationMode_str(str):
    """
    Returns a numerical value from a string
    """
    if str == "Waypoint": return NavigationMode.Waypoint
    if str == "Loiter": return NavigationMode.Loiter
    if str == "FlightDirector": return NavigationMode.FlightDirector
    if str == "TargetTrack": return NavigationMode.TargetTrack
    if str == "FollowLeader": return NavigationMode.FollowLeader
    if str == "LostComm": return NavigationMode.LostComm


def get_NavigationMode_int(val):
    """
    Returns a string representation from an int
    """
    if val == NavigationMode.Waypoint: return "Waypoint"
    if val == NavigationMode.Loiter: return "Loiter"
    if val == NavigationMode.FlightDirector: return "FlightDirector"
    if val == NavigationMode.TargetTrack: return "TargetTrack"
    if val == NavigationMode.FollowLeader: return "FollowLeader"
    if val == NavigationMode.LostComm: return "LostComm"
    return NavigationMode.Waypoint


