## ===============================================================================
## Authors: AFRL/RQQA
## Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
## 
## Copyright (c) 2017 Government of the United State of America, as represented by
## the Secretary of the Air Force.  No copyright is claimed in the United States under
## Title 17, U.S. Code.  All Other Rights Reserved.
## ===============================================================================

## This file was auto-created by LmcpGen. Modifications will be overwritten.

class ImpactPayloadType:

    Unknown = 0
    EO = 1
    FLIR = 2
    MWIR = 3
    LFIR = 4
    Track = 5
    Tag = 6
    Megaphone = 7
    Siren = 8
    SearchLight = 9
    FiftyCal = 10
    M240B = 11
    Flashbang = 12
    TearGas = 13
    Taser = 14
    HeatBeam = 15
    SEGM = 16
    CommRelay = 17
    GMTI = 18
    LaserDesignator = 19
    LWIR = 20



def get_ImpactPayloadType_str(str):
    """
    Returns a numerical value from a string
    """
    if str == "Unknown": return ImpactPayloadType.Unknown
    if str == "EO": return ImpactPayloadType.EO
    if str == "FLIR": return ImpactPayloadType.FLIR
    if str == "MWIR": return ImpactPayloadType.MWIR
    if str == "LFIR": return ImpactPayloadType.LFIR
    if str == "Track": return ImpactPayloadType.Track
    if str == "Tag": return ImpactPayloadType.Tag
    if str == "Megaphone": return ImpactPayloadType.Megaphone
    if str == "Siren": return ImpactPayloadType.Siren
    if str == "SearchLight": return ImpactPayloadType.SearchLight
    if str == "FiftyCal": return ImpactPayloadType.FiftyCal
    if str == "M240B": return ImpactPayloadType.M240B
    if str == "Flashbang": return ImpactPayloadType.Flashbang
    if str == "TearGas": return ImpactPayloadType.TearGas
    if str == "Taser": return ImpactPayloadType.Taser
    if str == "HeatBeam": return ImpactPayloadType.HeatBeam
    if str == "SEGM": return ImpactPayloadType.SEGM
    if str == "CommRelay": return ImpactPayloadType.CommRelay
    if str == "GMTI": return ImpactPayloadType.GMTI
    if str == "LaserDesignator": return ImpactPayloadType.LaserDesignator
    if str == "LWIR": return ImpactPayloadType.LWIR


def get_ImpactPayloadType_int(val):
    """
    Returns a string representation from an int
    """
    if val == ImpactPayloadType.Unknown: return "Unknown"
    if val == ImpactPayloadType.EO: return "EO"
    if val == ImpactPayloadType.FLIR: return "FLIR"
    if val == ImpactPayloadType.MWIR: return "MWIR"
    if val == ImpactPayloadType.LFIR: return "LFIR"
    if val == ImpactPayloadType.Track: return "Track"
    if val == ImpactPayloadType.Tag: return "Tag"
    if val == ImpactPayloadType.Megaphone: return "Megaphone"
    if val == ImpactPayloadType.Siren: return "Siren"
    if val == ImpactPayloadType.SearchLight: return "SearchLight"
    if val == ImpactPayloadType.FiftyCal: return "FiftyCal"
    if val == ImpactPayloadType.M240B: return "M240B"
    if val == ImpactPayloadType.Flashbang: return "Flashbang"
    if val == ImpactPayloadType.TearGas: return "TearGas"
    if val == ImpactPayloadType.Taser: return "Taser"
    if val == ImpactPayloadType.HeatBeam: return "HeatBeam"
    if val == ImpactPayloadType.SEGM: return "SEGM"
    if val == ImpactPayloadType.CommRelay: return "CommRelay"
    if val == ImpactPayloadType.GMTI: return "GMTI"
    if val == ImpactPayloadType.LaserDesignator: return "LaserDesignator"
    if val == ImpactPayloadType.LWIR: return "LWIR"
    return ImpactPayloadType.Unknown


