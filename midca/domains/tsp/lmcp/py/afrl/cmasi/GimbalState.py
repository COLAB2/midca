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

from midca.domains.tsp.lmcp.py.afrl.cmasi import PayloadState
from midca.domains.tsp.lmcp.py.afrl.cmasi import GimbalPointingMode


class GimbalState(PayloadState.PayloadState):

    def __init__(self):
        PayloadState.PayloadState.__init__(self)
        self.LMCP_TYPE = 27
        self.SERIES_NAME = "CMASI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.GimbalState"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 4849604199710720000
        self.SERIES_VERSION = 3

        #Define message fields
        self.PointingMode = GimbalPointingMode.GimbalPointingMode.Unknown   #GimbalPointingMode
        self.Azimuth = 0   #real32
        self.Elevation = 0   #real32
        self.Rotation = 0   #real32


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(PayloadState.PayloadState.pack(self))
        buffer.extend(struct.pack(">i", self.PointingMode))
        buffer.extend(struct.pack(">f", self.Azimuth))
        buffer.extend(struct.pack(">f", self.Elevation))
        buffer.extend(struct.pack(">f", self.Rotation))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = PayloadState.PayloadState.unpack(self, buffer, _pos)
        self.PointingMode = struct.unpack_from(">i", buffer, _pos)[0]
        _pos += 4
        self.Azimuth = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.Elevation = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.Rotation = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        PayloadState.PayloadState.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "PointingMode" and len(e.childNodes) > 0 :
                    self.PointingMode = GimbalPointingMode.get_GimbalPointingMode_str(e.childNodes[0].nodeValue)
                elif e.localName == "Azimuth" and len(e.childNodes) > 0 :
                    self.Azimuth = float(e.childNodes[0].nodeValue)
                elif e.localName == "Elevation" and len(e.childNodes) > 0 :
                    self.Elevation = float(e.childNodes[0].nodeValue)
                elif e.localName == "Rotation" and len(e.childNodes) > 0 :
                    self.Rotation = float(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        PayloadState.PayloadState.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "PointingMode":
                self.PointingMode = d[key]
            elif key == "Azimuth":
                self.Azimuth = d[key]
            elif key == "Elevation":
                self.Elevation = d[key]
            elif key == "Rotation":
                self.Rotation = d[key]

        return

    def get_PointingMode(self):
        return self.PointingMode

    def set_PointingMode(self, value):
        self.PointingMode = value 

    def get_Azimuth(self):
        return self.Azimuth

    def set_Azimuth(self, value):
        self.Azimuth = float( value )

    def get_Elevation(self):
        return self.Elevation

    def set_Elevation(self, value):
        self.Elevation = float( value )

    def get_Rotation(self):
        return self.Rotation

    def set_Rotation(self, value):
        self.Rotation = float( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = PayloadState.PayloadState.toString(self)
        buf += "From GimbalState:\n"
        buf +=    "PointingMode = " + str( self.PointingMode ) + "\n" 
        buf +=    "Azimuth = " + str( self.Azimuth ) + "\n" 
        buf +=    "Elevation = " + str( self.Elevation ) + "\n" 
        buf +=    "Rotation = " + str( self.Rotation ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "CMASI") or (len("CMASI") == 0): # this should never happen
        	# Checks for "CMASI" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/GimbalState")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("CMASI" + "/GimbalState")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        PayloadState.PayloadState.toDictMembers(self, d)
        d['PointingMode'] = self.PointingMode
        d['Azimuth'] = self.Azimuth
        d['Elevation'] = self.Elevation
        d['Rotation'] = self.Rotation

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
        str = ws + '<GimbalState Series="CMASI" >\n';
        #str +=PayloadState.PayloadState.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</GimbalState>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += PayloadState.PayloadState.toXMLMembersStr(self, ws)
        buf += ws + "<PointingMode>" + GimbalPointingMode.get_GimbalPointingMode_int(self.PointingMode) + "</PointingMode>\n"
        buf += ws + "<Azimuth>" + str(self.Azimuth) + "</Azimuth>\n"
        buf += ws + "<Elevation>" + str(self.Elevation) + "</Elevation>\n"
        buf += ws + "<Rotation>" + str(self.Rotation) + "</Rotation>\n"

        return buf
        
