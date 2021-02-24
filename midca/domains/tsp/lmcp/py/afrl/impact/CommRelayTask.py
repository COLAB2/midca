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

from midca.domains.tsp.lmcp.py.afrl.cmasi import Task
from midca.domains.tsp.lmcp.py.afrl.cmasi import Location3D


class CommRelayTask(Task.Task):

    def __init__(self):
        Task.Task.__init__(self)
        self.LMCP_TYPE = 28
        self.SERIES_NAME = "IMPACT"
        self.FULL_LMCP_TYPE_NAME = "afrl.impact.CommRelayTask"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5281966179208134656
        self.SERIES_VERSION = 14

        #Define message fields
        self.SupportedEntityID = 0   #int64
        self.DestinationLocation = None   #Location3D
        self.TowerID = 0   #int64


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(Task.Task.pack(self))
        buffer.extend(struct.pack(">q", self.SupportedEntityID))
        buffer.extend(struct.pack("B", self.DestinationLocation != None ))
        if self.DestinationLocation != None:
            buffer.extend(struct.pack(">q", self.DestinationLocation.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.DestinationLocation.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.DestinationLocation.SERIES_VERSION))
            buffer.extend(self.DestinationLocation.pack())
        buffer.extend(struct.pack(">q", self.TowerID))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = Task.Task.unpack(self, buffer, _pos)
        self.SupportedEntityID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        _valid = struct.unpack_from("B", buffer, _pos )[0]
        _pos += 1
        if _valid:
            _series = struct.unpack_from(">q", buffer, _pos)[0]
            _pos += 8
            _type = struct.unpack_from(">I", buffer, _pos)[0]
            _pos += 4
            _version = struct.unpack_from(">H", buffer, _pos)[0]
            _pos += 2
            from midca.domains.tsp.lmcp.py.lmcp import LMCPFactory
            self.DestinationLocation = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.DestinationLocation.unpack(buffer, _pos)
        else:
            self.DestinationLocation = None
        self.TowerID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        Task.Task.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "SupportedEntityID" and len(e.childNodes) > 0 :
                    self.SupportedEntityID = int(e.childNodes[0].nodeValue)
                elif e.localName == "DestinationLocation" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.DestinationLocation = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.DestinationLocation != None:
                                self.DestinationLocation.unpackFromXMLNode(n, seriesFactory)
                elif e.localName == "TowerID" and len(e.childNodes) > 0 :
                    self.TowerID = int(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        Task.Task.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "SupportedEntityID":
                self.SupportedEntityID = d[key]
            elif key == "DestinationLocation":
                self.DestinationLocation = seriesFactory.unpackFromDict(d[key])
            elif key == "TowerID":
                self.TowerID = d[key]

        return

    def get_SupportedEntityID(self):
        return self.SupportedEntityID

    def set_SupportedEntityID(self, value):
        self.SupportedEntityID = int( value )

    def get_DestinationLocation(self):
        return self.DestinationLocation

    def set_DestinationLocation(self, value):
        self.DestinationLocation = value 

    def get_TowerID(self):
        return self.TowerID

    def set_TowerID(self, value):
        self.TowerID = int( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = Task.Task.toString(self)
        buf += "From CommRelayTask:\n"
        buf +=    "SupportedEntityID = " + str( self.SupportedEntityID ) + "\n" 
        buf +=    "DestinationLocation = " + str( self.DestinationLocation ) + "\n" 
        buf +=    "TowerID = " + str( self.TowerID ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "IMPACT") or (len("IMPACT") == 0): # this should never happen
        	# Checks for "IMPACT" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/CommRelayTask")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("IMPACT" + "/CommRelayTask")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        Task.Task.toDictMembers(self, d)
        d['SupportedEntityID'] = self.SupportedEntityID
        if self.DestinationLocation == None:
            d['DestinationLocation'] = None
        else:
            d['DestinationLocation'] = self.DestinationLocation.toDict()
        d['TowerID'] = self.TowerID

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
        str = ws + '<CommRelayTask Series="IMPACT" >\n';
        #str +=Task.Task.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</CommRelayTask>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += Task.Task.toXMLMembersStr(self, ws)
        buf += ws + "<SupportedEntityID>" + str(self.SupportedEntityID) + "</SupportedEntityID>\n"
        if self.DestinationLocation != None:
            buf += ws + "<DestinationLocation>\n"
            buf += ws + self.DestinationLocation.toXMLStr(ws + "    ") 
            buf += ws + "</DestinationLocation>\n"
        buf += ws + "<TowerID>" + str(self.TowerID) + "</TowerID>\n"

        return buf
        
