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


class BlockadeTask(Task.Task):

    def __init__(self):
        Task.Task.__init__(self)
        self.LMCP_TYPE = 30
        self.SERIES_NAME = "IMPACT"
        self.FULL_LMCP_TYPE_NAME = "afrl.impact.BlockadeTask"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5281966179208134656
        self.SERIES_VERSION = 14

        #Define message fields
        self.BlockedEntityID = 0   #int64
        self.StandoffDistance = 0   #real32
        self.NumberVehicles = 1   #byte
        self.ProtectedLocation = None   #Location3D


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(Task.Task.pack(self))
        buffer.extend(struct.pack(">q", self.BlockedEntityID))
        buffer.extend(struct.pack(">f", self.StandoffDistance))
        buffer.extend(struct.pack(">B", self.NumberVehicles))
        buffer.extend(struct.pack("B", self.ProtectedLocation != None ))
        if self.ProtectedLocation != None:
            buffer.extend(struct.pack(">q", self.ProtectedLocation.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.ProtectedLocation.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.ProtectedLocation.SERIES_VERSION))
            buffer.extend(self.ProtectedLocation.pack())

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = Task.Task.unpack(self, buffer, _pos)
        self.BlockedEntityID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.StandoffDistance = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.NumberVehicles = struct.unpack_from(">B", buffer, _pos)[0]
        _pos += 1
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
            self.ProtectedLocation = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.ProtectedLocation.unpack(buffer, _pos)
        else:
            self.ProtectedLocation = None
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        Task.Task.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "BlockedEntityID" and len(e.childNodes) > 0 :
                    self.BlockedEntityID = int(e.childNodes[0].nodeValue)
                elif e.localName == "StandoffDistance" and len(e.childNodes) > 0 :
                    self.StandoffDistance = float(e.childNodes[0].nodeValue)
                elif e.localName == "NumberVehicles" and len(e.childNodes) > 0 :
                    self.NumberVehicles = int(e.childNodes[0].nodeValue)
                elif e.localName == "ProtectedLocation" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.ProtectedLocation = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.ProtectedLocation != None:
                                self.ProtectedLocation.unpackFromXMLNode(n, seriesFactory)

        return

    def unpackFromDict(self, d, seriesFactory):
        Task.Task.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "BlockedEntityID":
                self.BlockedEntityID = d[key]
            elif key == "StandoffDistance":
                self.StandoffDistance = d[key]
            elif key == "NumberVehicles":
                self.NumberVehicles = d[key]
            elif key == "ProtectedLocation":
                self.ProtectedLocation = seriesFactory.unpackFromDict(d[key])

        return

    def get_BlockedEntityID(self):
        return self.BlockedEntityID

    def set_BlockedEntityID(self, value):
        self.BlockedEntityID = int( value )

    def get_StandoffDistance(self):
        return self.StandoffDistance

    def set_StandoffDistance(self, value):
        self.StandoffDistance = float( value )

    def get_NumberVehicles(self):
        return self.NumberVehicles

    def set_NumberVehicles(self, value):
        self.NumberVehicles = int( value )

    def get_ProtectedLocation(self):
        return self.ProtectedLocation

    def set_ProtectedLocation(self, value):
        self.ProtectedLocation = value 



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = Task.Task.toString(self)
        buf += "From BlockadeTask:\n"
        buf +=    "BlockedEntityID = " + str( self.BlockedEntityID ) + "\n" 
        buf +=    "StandoffDistance = " + str( self.StandoffDistance ) + "\n" 
        buf +=    "NumberVehicles = " + str( self.NumberVehicles ) + "\n" 
        buf +=    "ProtectedLocation = " + str( self.ProtectedLocation ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "IMPACT") or (len("IMPACT") == 0): # this should never happen
        	# Checks for "IMPACT" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/BlockadeTask")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("IMPACT" + "/BlockadeTask")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        Task.Task.toDictMembers(self, d)
        d['BlockedEntityID'] = self.BlockedEntityID
        d['StandoffDistance'] = self.StandoffDistance
        d['NumberVehicles'] = self.NumberVehicles
        if self.ProtectedLocation == None:
            d['ProtectedLocation'] = None
        else:
            d['ProtectedLocation'] = self.ProtectedLocation.toDict()

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
        str = ws + '<BlockadeTask Series="IMPACT" >\n';
        #str +=Task.Task.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</BlockadeTask>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += Task.Task.toXMLMembersStr(self, ws)
        buf += ws + "<BlockedEntityID>" + str(self.BlockedEntityID) + "</BlockedEntityID>\n"
        buf += ws + "<StandoffDistance>" + str(self.StandoffDistance) + "</StandoffDistance>\n"
        buf += ws + "<NumberVehicles>" + str(self.NumberVehicles) + "</NumberVehicles>\n"
        if self.ProtectedLocation != None:
            buf += ws + "<ProtectedLocation>\n"
            buf += ws + self.ProtectedLocation.toXMLStr(ws + "    ") 
            buf += ws + "</ProtectedLocation>\n"

        return buf
        
