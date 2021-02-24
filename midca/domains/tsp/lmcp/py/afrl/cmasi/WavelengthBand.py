## ===============================================================================
## Authors: AFRL/RQQA
## Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
## 
## Copyright (c) 2017 Government of the United State of America, as represented by
## the Secretary of the Air Force.  No copyright is claimed in the United States under
## Title 17, U.S. Code.  All Other Rights Reserved.
## ===============================================================================

## This file was auto-created by LmcpGen. Modifications will be overwritten.

class WavelengthBand:

    AllAny = 0
    EO = 1
    LWIR = 2
    SWIR = 3
    MWIR = 4
    Other = 5



def get_WavelengthBand_str(str):
    """
    Returns a numerical value from a string
    """
    if str == "AllAny": return WavelengthBand.AllAny
    if str == "EO": return WavelengthBand.EO
    if str == "LWIR": return WavelengthBand.LWIR
    if str == "SWIR": return WavelengthBand.SWIR
    if str == "MWIR": return WavelengthBand.MWIR
    if str == "Other": return WavelengthBand.Other


def get_WavelengthBand_int(val):
    """
    Returns a string representation from an int
    """
    if val == WavelengthBand.AllAny: return "AllAny"
    if val == WavelengthBand.EO: return "EO"
    if val == WavelengthBand.LWIR: return "LWIR"
    if val == WavelengthBand.SWIR: return "SWIR"
    if val == WavelengthBand.MWIR: return "MWIR"
    if val == WavelengthBand.Other: return "Other"
    return WavelengthBand.AllAny


