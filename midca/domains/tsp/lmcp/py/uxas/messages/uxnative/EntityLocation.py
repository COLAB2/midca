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

from midca.domains.tsp.lmcp.py.afrl.cmasi import Location3D


class EntityLocation(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 7
        self.SERIES_NAME = "UXNATIVE"
        self.FULL_LMCP_TYPE_NAME = "uxas.messages.uxnative.EntityLocation"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 6149751333668345413
        self.SERIES_VERSION = 9

        #Define message fields
        self.EntityID = 0   #int64
        self.Position = Location3D.Location3D()   #Location3D
        self.Time = 0   #int64


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.EntityID))
        buffer.extend(struct.pack("B", self.Position != None ))
        if self.Position != None:
            buffer.extend(struct.pack(">q", self.Position.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.Position.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.Position.SERIES_VERSION))
            buffer.extend(self.Position.pack())
        buffer.extend(struct.pack(">q", self.Time))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.EntityID = struct.unpack_from(">q", buffer, _pos)[0]
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
            self.Position = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.Position.unpack(buffer, _pos)
        else:
            self.Position = None
        self.Time = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "EntityID" and len(e.childNodes) > 0 :
                    self.EntityID = int(e.childNodes[0].nodeValue)
                elif e.localName == "Position" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.Position = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.Position != None:
                                self.Position.unpackFromXMLNode(n, seriesFactory)
                elif e.localName == "Time" and len(e.childNodes) > 0 :
                    self.Time = int(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "EntityID":
                self.EntityID = d[key]
            elif key == "Position":
                self.Position = seriesFactory.unpackFromDict(d[key])
            elif key == "Time":
                self.Time = d[key]

        return

    def get_EntityID(self):
        return self.EntityID

    def set_EntityID(self, value):
        self.EntityID = int( value )

    def get_Position(self):
        return self.Position

    def set_Position(self, value):
        self.Position = value 

    def get_Time(self):
        return self.Time

    def set_Time(self, value):
        self.Time = int( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From EntityLocation:\n"
        buf +=    "EntityID = " + str( self.EntityID ) + "\n" 
        buf +=    "Position = " + str( self.Position ) + "\n" 
        buf +=    "Time = " + str( self.Time ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "UXNATIVE") or (len("UXNATIVE") == 0): # this should never happen
        	# Checks for "UXNATIVE" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/EntityLocation")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("UXNATIVE" + "/EntityLocation")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['EntityID'] = self.EntityID
        if self.Position == None:
            d['Position'] = None
        else:
            d['Position'] = self.Position.toDict()
        d['Time'] = self.Time

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
        str = ws + '<EntityLocation Series="UXNATIVE" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</EntityLocation>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<EntityID>" + str(self.EntityID) + "</EntityID>\n"
        if self.Position != None:
            buf += ws + "<Position>\n"
            buf += ws + self.Position.toXMLStr(ws + "    ") 
            buf += ws + "</Position>\n"
        buf += ws + "<Time>" + str(self.Time) + "</Time>\n"

        return buf
        
