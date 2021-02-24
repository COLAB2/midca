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

from midca.domains.tsp.lmcp.py.afrl.cmasi import EntityConfiguration


class StationarySensorConfiguration(EntityConfiguration.EntityConfiguration):

    def __init__(self):
        EntityConfiguration.EntityConfiguration.__init__(self)
        self.LMCP_TYPE = 5
        self.SERIES_NAME = "VEHICLES"
        self.FULL_LMCP_TYPE_NAME = "afrl.vehicles.StationarySensorConfiguration"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 6216454340153722195
        self.SERIES_VERSION = 1

        #Define message fields


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(EntityConfiguration.EntityConfiguration.pack(self))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = EntityConfiguration.EntityConfiguration.unpack(self, buffer, _pos)
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        EntityConfiguration.EntityConfiguration.unpackFromXMLNode(self, el, seriesFactory)

        return

    def unpackFromDict(self, d, seriesFactory):
        EntityConfiguration.EntityConfiguration.unpackFromDict(self, d, seriesFactory)

        return



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = EntityConfiguration.EntityConfiguration.toString(self)
        buf += "From StationarySensorConfiguration:\n"

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "VEHICLES") or (len("VEHICLES") == 0): # this should never happen
        	# Checks for "VEHICLES" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/StationarySensorConfiguration")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("VEHICLES" + "/StationarySensorConfiguration")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        EntityConfiguration.EntityConfiguration.toDictMembers(self, d)

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
        str = ws + '<StationarySensorConfiguration Series="VEHICLES" >\n';
        #str +=EntityConfiguration.EntityConfiguration.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</StationarySensorConfiguration>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += EntityConfiguration.EntityConfiguration.toXMLMembersStr(self, ws)

        return buf
        
