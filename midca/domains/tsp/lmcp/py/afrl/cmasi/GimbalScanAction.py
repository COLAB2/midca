#! /usr/bin/python

import sys, struct
import xml.dom.minidom
from midca.domains.tsp.lmcp.py.lmcp import LMCPObject

## ===============================================================================
## Authors: AFRL/RQQA
## Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
## 
## Copyright (c) 2017 Government of the United State of America, as represented by
## the Secretary of the Air Force.  No copyright is claimed in the United States under
## Title 17, U.S. Code.  All Other Rights Reserved.
## ===============================================================================

## This file was auto-created by LmcpGen. Modifications will be overwritten.

from midca.domains.tsp.lmcp.py.afrl.cmasi import PayloadAction


class GimbalScanAction(PayloadAction.PayloadAction):

    def __init__(self):
        PayloadAction.PayloadAction.__init__(self)
        self.LMCP_TYPE = 25
        self.SERIES_NAME = "CMASI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.GimbalScanAction"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 4849604199710720000
        self.SERIES_VERSION = 3

        #Define message fields
        self.AzimuthSlewRate = 0   #real32
        self.ElevationSlewRate = 0   #real32
        self.StartAzimuth = 0   #real32
        self.EndAzimuth = 0   #real32
        self.StartElevation = 0   #real32
        self.EndElevation = 0   #real32
        self.Cycles = 0   #uint32


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(PayloadAction.PayloadAction.pack(self))
        buffer.extend(struct.pack(">f", self.AzimuthSlewRate))
        buffer.extend(struct.pack(">f", self.ElevationSlewRate))
        buffer.extend(struct.pack(">f", self.StartAzimuth))
        buffer.extend(struct.pack(">f", self.EndAzimuth))
        buffer.extend(struct.pack(">f", self.StartElevation))
        buffer.extend(struct.pack(">f", self.EndElevation))
        buffer.extend(struct.pack(">I", self.Cycles))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = PayloadAction.PayloadAction.unpack(self, buffer, _pos)
        self.AzimuthSlewRate = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.ElevationSlewRate = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.StartAzimuth = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.EndAzimuth = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.StartElevation = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.EndElevation = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.Cycles = struct.unpack_from(">I", buffer, _pos)[0]
        _pos += 4
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        PayloadAction.PayloadAction.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "AzimuthSlewRate" and len(e.childNodes) > 0 :
                    self.AzimuthSlewRate = float(e.childNodes[0].nodeValue)
                elif e.localName == "ElevationSlewRate" and len(e.childNodes) > 0 :
                    self.ElevationSlewRate = float(e.childNodes[0].nodeValue)
                elif e.localName == "StartAzimuth" and len(e.childNodes) > 0 :
                    self.StartAzimuth = float(e.childNodes[0].nodeValue)
                elif e.localName == "EndAzimuth" and len(e.childNodes) > 0 :
                    self.EndAzimuth = float(e.childNodes[0].nodeValue)
                elif e.localName == "StartElevation" and len(e.childNodes) > 0 :
                    self.StartElevation = float(e.childNodes[0].nodeValue)
                elif e.localName == "EndElevation" and len(e.childNodes) > 0 :
                    self.EndElevation = float(e.childNodes[0].nodeValue)
                elif e.localName == "Cycles" and len(e.childNodes) > 0 :
                    self.Cycles = int(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        PayloadAction.PayloadAction.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "AzimuthSlewRate":
                self.AzimuthSlewRate = d[key]
            elif key == "ElevationSlewRate":
                self.ElevationSlewRate = d[key]
            elif key == "StartAzimuth":
                self.StartAzimuth = d[key]
            elif key == "EndAzimuth":
                self.EndAzimuth = d[key]
            elif key == "StartElevation":
                self.StartElevation = d[key]
            elif key == "EndElevation":
                self.EndElevation = d[key]
            elif key == "Cycles":
                self.Cycles = d[key]

        return

    def get_AzimuthSlewRate(self):
        return self.AzimuthSlewRate

    def set_AzimuthSlewRate(self, value):
        self.AzimuthSlewRate = float( value )

    def get_ElevationSlewRate(self):
        return self.ElevationSlewRate

    def set_ElevationSlewRate(self, value):
        self.ElevationSlewRate = float( value )

    def get_StartAzimuth(self):
        return self.StartAzimuth

    def set_StartAzimuth(self, value):
        self.StartAzimuth = float( value )

    def get_EndAzimuth(self):
        return self.EndAzimuth

    def set_EndAzimuth(self, value):
        self.EndAzimuth = float( value )

    def get_StartElevation(self):
        return self.StartElevation

    def set_StartElevation(self, value):
        self.StartElevation = float( value )

    def get_EndElevation(self):
        return self.EndElevation

    def set_EndElevation(self, value):
        self.EndElevation = float( value )

    def get_Cycles(self):
        return self.Cycles

    def set_Cycles(self, value):
        self.Cycles = int( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = PayloadAction.PayloadAction.toString(self)
        buf += "From GimbalScanAction:\n"
        buf +=    "AzimuthSlewRate = " + str( self.AzimuthSlewRate ) + "\n" 
        buf +=    "ElevationSlewRate = " + str( self.ElevationSlewRate ) + "\n" 
        buf +=    "StartAzimuth = " + str( self.StartAzimuth ) + "\n" 
        buf +=    "EndAzimuth = " + str( self.EndAzimuth ) + "\n" 
        buf +=    "StartElevation = " + str( self.StartElevation ) + "\n" 
        buf +=    "EndElevation = " + str( self.EndElevation ) + "\n" 
        buf +=    "Cycles = " + str( self.Cycles ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "CMASI") or (len("CMASI") == 0): # this should never happen
        	# Checks for "CMASI" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/GimbalScanAction")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("CMASI" + "/GimbalScanAction")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        PayloadAction.PayloadAction.toDictMembers(self, d)
        d['AzimuthSlewRate'] = self.AzimuthSlewRate
        d['ElevationSlewRate'] = self.ElevationSlewRate
        d['StartAzimuth'] = self.StartAzimuth
        d['EndAzimuth'] = self.EndAzimuth
        d['StartElevation'] = self.StartElevation
        d['EndElevation'] = self.EndElevation
        d['Cycles'] = self.Cycles

        return

    def getLMCPType(self):
        return self.LMCP_TYPE

    def getSeriesName(self):
        return self.SERIES_NAME

    def getSeriesNameID(self):
        return self.SERIES_NAME_ID

    def getSeriesVersion(self):
        return self.SERIES_VERSION

    def toXMLStr(self, ws):
        str = ws + '<GimbalScanAction Series="CMASI" >\n';
        #str +=PayloadAction.PayloadAction.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</GimbalScanAction>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += PayloadAction.PayloadAction.toXMLMembersStr(self, ws)
        buf += ws + "<AzimuthSlewRate>" + str(self.AzimuthSlewRate) + "</AzimuthSlewRate>\n"
        buf += ws + "<ElevationSlewRate>" + str(self.ElevationSlewRate) + "</ElevationSlewRate>\n"
        buf += ws + "<StartAzimuth>" + str(self.StartAzimuth) + "</StartAzimuth>\n"
        buf += ws + "<EndAzimuth>" + str(self.EndAzimuth) + "</EndAzimuth>\n"
        buf += ws + "<StartElevation>" + str(self.StartElevation) + "</StartElevation>\n"
        buf += ws + "<EndElevation>" + str(self.EndElevation) + "</EndElevation>\n"
        buf += ws + "<Cycles>" + str(self.Cycles) + "</Cycles>\n"

        return buf
        
