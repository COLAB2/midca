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

from midca.domains.tsp.lmcp.py.afrl.cmasi import AbstractZone
from midca.domains.tsp.lmcp.py.afrl.cmasi import ZoneAvoidanceType


class KeepOutZone(AbstractZone.AbstractZone):

    def __init__(self):
        AbstractZone.AbstractZone.__init__(self)
        self.LMCP_TYPE = 30
        self.SERIES_NAME = "CMASI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.KeepOutZone"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 4849604199710720000
        self.SERIES_VERSION = 3

        #Define message fields
        self.ZoneType = ZoneAvoidanceType.ZoneAvoidanceType.Physical   #ZoneAvoidanceType


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(AbstractZone.AbstractZone.pack(self))
        buffer.extend(struct.pack(">i", self.ZoneType))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = AbstractZone.AbstractZone.unpack(self, buffer, _pos)
        self.ZoneType = struct.unpack_from(">i", buffer, _pos)[0]
        _pos += 4
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        AbstractZone.AbstractZone.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "ZoneType" and len(e.childNodes) > 0 :
                    self.ZoneType = ZoneAvoidanceType.get_ZoneAvoidanceType_str(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        AbstractZone.AbstractZone.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "ZoneType":
                self.ZoneType = d[key]

        return

    def get_ZoneType(self):
        return self.ZoneType

    def set_ZoneType(self, value):
        self.ZoneType = value 



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = AbstractZone.AbstractZone.toString(self)
        buf += "From KeepOutZone:\n"
        buf +=    "ZoneType = " + str( self.ZoneType ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "CMASI") or (len("CMASI") == 0): # this should never happen
        	# Checks for "CMASI" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/KeepOutZone")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("CMASI" + "/KeepOutZone")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        AbstractZone.AbstractZone.toDictMembers(self, d)
        d['ZoneType'] = self.ZoneType

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
        str = ws + '<KeepOutZone Series="CMASI" >\n';
        #str +=AbstractZone.AbstractZone.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</KeepOutZone>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += AbstractZone.AbstractZone.toXMLMembersStr(self, ws)
        buf += ws + "<ZoneType>" + ZoneAvoidanceType.get_ZoneAvoidanceType_int(self.ZoneType) + "</ZoneType>\n"

        return buf
        
