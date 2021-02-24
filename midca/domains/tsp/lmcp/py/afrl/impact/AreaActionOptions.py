## ===============================================================================
## Authors: AFRL/RQQA
## Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
## 
## Copyright (c) 2017 Government of the United State of America, as represented by
## the Secretary of the Air Force.  No copyright is claimed in the United States under
## Title 17, U.S. Code.  All Other Rights Reserved.
## ===============================================================================

## This file was auto-created by LmcpGen. Modifications will be overwritten.

class AreaActionOptions:

    Created = 0
    Destroyed = 1
    Modified = 2



def get_AreaActionOptions_str(str):
    """
    Returns a numerical value from a string
    """
    if str == "Created": return AreaActionOptions.Created
    if str == "Destroyed": return AreaActionOptions.Destroyed
    if str == "Modified": return AreaActionOptions.Modified


def get_AreaActionOptions_int(val):
    """
    Returns a string representation from an int
    """
    if val == AreaActionOptions.Created: return "Created"
    if val == AreaActionOptions.Destroyed: return "Destroyed"
    if val == AreaActionOptions.Modified: return "Modified"
    return AreaActionOptions.Created


