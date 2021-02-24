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

from midca.domains.tsp.lmcp.py.afrl.cmasi import VehicleAction


class TrackEntityAction(VehicleAction.VehicleAction):

    def __init__(self):
        VehicleAction.VehicleAction.__init__(self)
        self.LMCP_TYPE = 2
        self.SERIES_NAME = "PERCEIVE"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.perceive.TrackEntityAction"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5784119745305990725
        self.SERIES_VERSION = 1

        #Define message fields
        self.EntityID = 0   #uint32
        self.SensorID = 0   #uint32
        self.ReturnToWaypoint = 0   #uint32


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(VehicleAction.VehicleAction.pack(self))
        buffer.extend(struct.pack(">I", self.EntityID))
        buffer.extend(struct.pack(">I", self.SensorID))
        buffer.extend(struct.pack(">I", self.ReturnToWaypoint))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = VehicleAction.VehicleAction.unpack(self, buffer, _pos)
        self.EntityID = struct.unpack_from(">I", buffer, _pos)[0]
        _pos += 4
        self.SensorID = struct.unpack_from(">I", buffer, _pos)[0]
        _pos += 4
        self.ReturnToWaypoint = struct.unpack_from(">I", buffer, _pos)[0]
        _pos += 4
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        VehicleAction.VehicleAction.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "EntityID" and len(e.childNodes) > 0 :
                    self.EntityID = int(e.childNodes[0].nodeValue)
                elif e.localName == "SensorID" and len(e.childNodes) > 0 :
                    self.SensorID = int(e.childNodes[0].nodeValue)
                elif e.localName == "ReturnToWaypoint" and len(e.childNodes) > 0 :
                    self.ReturnToWaypoint = int(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        VehicleAction.VehicleAction.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "EntityID":
                self.EntityID = d[key]
            elif key == "SensorID":
                self.SensorID = d[key]
            elif key == "ReturnToWaypoint":
                self.ReturnToWaypoint = d[key]

        return

    def get_EntityID(self):
        return self.EntityID

    def set_EntityID(self, value):
        self.EntityID = int( value )

    def get_SensorID(self):
        return self.SensorID

    def set_SensorID(self, value):
        self.SensorID = int( value )

    def get_ReturnToWaypoint(self):
        return self.ReturnToWaypoint

    def set_ReturnToWaypoint(self, value):
        self.ReturnToWaypoint = int( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = VehicleAction.VehicleAction.toString(self)
        buf += "From TrackEntityAction:\n"
        buf +=    "EntityID = " + str( self.EntityID ) + "\n" 
        buf +=    "SensorID = " + str( self.SensorID ) + "\n" 
        buf +=    "ReturnToWaypoint = " + str( self.ReturnToWaypoint ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "PERCEIVE") or (len("PERCEIVE") == 0): # this should never happen
        	# Checks for "PERCEIVE" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/TrackEntityAction")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("PERCEIVE" + "/TrackEntityAction")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        VehicleAction.VehicleAction.toDictMembers(self, d)
        d['EntityID'] = self.EntityID
        d['SensorID'] = self.SensorID
        d['ReturnToWaypoint'] = self.ReturnToWaypoint

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
        str = ws + '<TrackEntityAction Series="PERCEIVE" >\n';
        #str +=VehicleAction.VehicleAction.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</TrackEntityAction>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += VehicleAction.VehicleAction.toXMLMembersStr(self, ws)
        buf += ws + "<EntityID>" + str(self.EntityID) + "</EntityID>\n"
        buf += ws + "<SensorID>" + str(self.SensorID) + "</SensorID>\n"
        buf += ws + "<ReturnToWaypoint>" + str(self.ReturnToWaypoint) + "</ReturnToWaypoint>\n"

        return buf
        
