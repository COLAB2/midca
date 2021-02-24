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



class Wedge(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 16
        self.SERIES_NAME = "CMASI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.Wedge"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 4849604199710720000
        self.SERIES_VERSION = 3

        #Define message fields
        self.AzimuthCenterline = 0   #real32
        self.VerticalCenterline = 0   #real32
        self.AzimuthExtent = 0   #real32
        self.VerticalExtent = 0   #real32


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">f", self.AzimuthCenterline))
        buffer.extend(struct.pack(">f", self.VerticalCenterline))
        buffer.extend(struct.pack(">f", self.AzimuthExtent))
        buffer.extend(struct.pack(">f", self.VerticalExtent))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.AzimuthCenterline = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.VerticalCenterline = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.AzimuthExtent = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.VerticalExtent = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "AzimuthCenterline" and len(e.childNodes) > 0 :
                    self.AzimuthCenterline = float(e.childNodes[0].nodeValue)
                elif e.localName == "VerticalCenterline" and len(e.childNodes) > 0 :
                    self.VerticalCenterline = float(e.childNodes[0].nodeValue)
                elif e.localName == "AzimuthExtent" and len(e.childNodes) > 0 :
                    self.AzimuthExtent = float(e.childNodes[0].nodeValue)
                elif e.localName == "VerticalExtent" and len(e.childNodes) > 0 :
                    self.VerticalExtent = float(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "AzimuthCenterline":
                self.AzimuthCenterline = d[key]
            elif key == "VerticalCenterline":
                self.VerticalCenterline = d[key]
            elif key == "AzimuthExtent":
                self.AzimuthExtent = d[key]
            elif key == "VerticalExtent":
                self.VerticalExtent = d[key]

        return

    def get_AzimuthCenterline(self):
        return self.AzimuthCenterline

    def set_AzimuthCenterline(self, value):
        self.AzimuthCenterline = float( value )

    def get_VerticalCenterline(self):
        return self.VerticalCenterline

    def set_VerticalCenterline(self, value):
        self.VerticalCenterline = float( value )

    def get_AzimuthExtent(self):
        return self.AzimuthExtent

    def set_AzimuthExtent(self, value):
        self.AzimuthExtent = float( value )

    def get_VerticalExtent(self):
        return self.VerticalExtent

    def set_VerticalExtent(self, value):
        self.VerticalExtent = float( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From Wedge:\n"
        buf +=    "AzimuthCenterline = " + str( self.AzimuthCenterline ) + "\n" 
        buf +=    "VerticalCenterline = " + str( self.VerticalCenterline ) + "\n" 
        buf +=    "AzimuthExtent = " + str( self.AzimuthExtent ) + "\n" 
        buf +=    "VerticalExtent = " + str( self.VerticalExtent ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "CMASI") or (len("CMASI") == 0): # this should never happen
        	# Checks for "CMASI" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/Wedge")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("CMASI" + "/Wedge")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['AzimuthCenterline'] = self.AzimuthCenterline
        d['VerticalCenterline'] = self.VerticalCenterline
        d['AzimuthExtent'] = self.AzimuthExtent
        d['VerticalExtent'] = self.VerticalExtent

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
        str = ws + '<Wedge Series="CMASI" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</Wedge>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<AzimuthCenterline>" + str(self.AzimuthCenterline) + "</AzimuthCenterline>\n"
        buf += ws + "<VerticalCenterline>" + str(self.VerticalCenterline) + "</VerticalCenterline>\n"
        buf += ws + "<AzimuthExtent>" + str(self.AzimuthExtent) + "</AzimuthExtent>\n"
        buf += ws + "<VerticalExtent>" + str(self.VerticalExtent) + "</VerticalExtent>\n"

        return buf
        
