## ===============================================================================
## Authors: AFRL/RQQA
## Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
## 
## Copyright (c) 2017 Government of the United State of America, as represented by
## the Secretary of the Air Force.  No copyright is claimed in the United States under
## Title 17, U.S. Code.  All Other Rights Reserved.
## ===============================================================================

## This file was auto-created by LmcpGen. Modifications will be overwritten.

class PowerPlant:

    Gasoline = 0
    JP5 = 1
    JP8 = 2
    FuelCell = 3
    Hybrid = 4
    Electric = 5



def get_PowerPlant_str(str):
    """
    Returns a numerical value from a string
    """
    if str == "Gasoline": return PowerPlant.Gasoline
    if str == "JP5": return PowerPlant.JP5
    if str == "JP8": return PowerPlant.JP8
    if str == "FuelCell": return PowerPlant.FuelCell
    if str == "Hybrid": return PowerPlant.Hybrid
    if str == "Electric": return PowerPlant.Electric


def get_PowerPlant_int(val):
    """
    Returns a string representation from an int
    """
    if val == PowerPlant.Gasoline: return "Gasoline"
    if val == PowerPlant.JP5: return "JP5"
    if val == PowerPlant.JP8: return "JP8"
    if val == PowerPlant.FuelCell: return "FuelCell"
    if val == PowerPlant.Hybrid: return "Hybrid"
    if val == PowerPlant.Electric: return "Electric"
    return PowerPlant.Gasoline


