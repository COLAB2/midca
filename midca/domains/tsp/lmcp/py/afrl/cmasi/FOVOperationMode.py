## ===============================================================================
## Authors: AFRL/RQQA
## Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
## 
## Copyright (c) 2017 Government of the United State of America, as represented by
## the Secretary of the Air Force.  No copyright is claimed in the United States under
## Title 17, U.S. Code.  All Other Rights Reserved.
## ===============================================================================

## This file was auto-created by LmcpGen. Modifications will be overwritten.

class FOVOperationMode:

    Continuous = 0
    Discrete = 1



def get_FOVOperationMode_str(str):
    """
    Returns a numerical value from a string
    """
    if str == "Continuous": return FOVOperationMode.Continuous
    if str == "Discrete": return FOVOperationMode.Discrete


def get_FOVOperationMode_int(val):
    """
    Returns a string representation from an int
    """
    if val == FOVOperationMode.Continuous: return "Continuous"
    if val == FOVOperationMode.Discrete: return "Discrete"
    return FOVOperationMode.Continuous


