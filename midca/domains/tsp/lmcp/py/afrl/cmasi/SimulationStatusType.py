## ===============================================================================
## Authors: AFRL/RQQA
## Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
## 
## Copyright (c) 2017 Government of the United State of America, as represented by
## the Secretary of the Air Force.  No copyright is claimed in the United States under
## Title 17, U.S. Code.  All Other Rights Reserved.
## ===============================================================================

## This file was auto-created by LmcpGen. Modifications will be overwritten.

class SimulationStatusType:

    Stopped = 0
    Running = 1
    Paused = 2
    Reset = 3



def get_SimulationStatusType_str(str):
    """
    Returns a numerical value from a string
    """
    if str == "Stopped": return SimulationStatusType.Stopped
    if str == "Running": return SimulationStatusType.Running
    if str == "Paused": return SimulationStatusType.Paused
    if str == "Reset": return SimulationStatusType.Reset


def get_SimulationStatusType_int(val):
    """
    Returns a string representation from an int
    """
    if val == SimulationStatusType.Stopped: return "Stopped"
    if val == SimulationStatusType.Running: return "Running"
    if val == SimulationStatusType.Paused: return "Paused"
    if val == SimulationStatusType.Reset: return "Reset"
    return SimulationStatusType.Stopped


