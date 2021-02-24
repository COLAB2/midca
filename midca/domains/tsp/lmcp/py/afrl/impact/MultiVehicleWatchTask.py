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

from midca.domains.tsp.lmcp.py.afrl.cmasi import SearchTask


class MultiVehicleWatchTask(SearchTask.SearchTask):

    def __init__(self):
        SearchTask.SearchTask.__init__(self)
        self.LMCP_TYPE = 27
        self.SERIES_NAME = "IMPACT"
        self.FULL_LMCP_TYPE_NAME = "afrl.impact.MultiVehicleWatchTask"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5281966179208134656
        self.SERIES_VERSION = 14

        #Define message fields
        self.WatchedEntityID = 0   #int64
        self.NumberVehicles = 1   #byte


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(SearchTask.SearchTask.pack(self))
        buffer.extend(struct.pack(">q", self.WatchedEntityID))
        buffer.extend(struct.pack(">B", self.NumberVehicles))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = SearchTask.SearchTask.unpack(self, buffer, _pos)
        self.WatchedEntityID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.NumberVehicles = struct.unpack_from(">B", buffer, _pos)[0]
        _pos += 1
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        SearchTask.SearchTask.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "WatchedEntityID" and len(e.childNodes) > 0 :
                    self.WatchedEntityID = int(e.childNodes[0].nodeValue)
                elif e.localName == "NumberVehicles" and len(e.childNodes) > 0 :
                    self.NumberVehicles = int(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        SearchTask.SearchTask.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "WatchedEntityID":
                self.WatchedEntityID = d[key]
            elif key == "NumberVehicles":
                self.NumberVehicles = d[key]

        return

    def get_WatchedEntityID(self):
        return self.WatchedEntityID

    def set_WatchedEntityID(self, value):
        self.WatchedEntityID = int( value )

    def get_NumberVehicles(self):
        return self.NumberVehicles

    def set_NumberVehicles(self, value):
        self.NumberVehicles = int( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = SearchTask.SearchTask.toString(self)
        buf += "From MultiVehicleWatchTask:\n"
        buf +=    "WatchedEntityID = " + str( self.WatchedEntityID ) + "\n" 
        buf +=    "NumberVehicles = " + str( self.NumberVehicles ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "IMPACT") or (len("IMPACT") == 0): # this should never happen
        	# Checks for "IMPACT" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/MultiVehicleWatchTask")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("IMPACT" + "/MultiVehicleWatchTask")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        SearchTask.SearchTask.toDictMembers(self, d)
        d['WatchedEntityID'] = self.WatchedEntityID
        d['NumberVehicles'] = self.NumberVehicles

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
        str = ws + '<MultiVehicleWatchTask Series="IMPACT" >\n';
        #str +=SearchTask.SearchTask.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</MultiVehicleWatchTask>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += SearchTask.SearchTask.toXMLMembersStr(self, ws)
        buf += ws + "<WatchedEntityID>" + str(self.WatchedEntityID) + "</WatchedEntityID>\n"
        buf += ws + "<NumberVehicles>" + str(self.NumberVehicles) + "</NumberVehicles>\n"

        return buf
        
