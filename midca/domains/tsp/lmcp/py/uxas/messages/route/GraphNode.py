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


class GraphNode(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 1
        self.SERIES_NAME = "ROUTE"
        self.FULL_LMCP_TYPE_NAME = "uxas.messages.route.GraphNode"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5931053054693474304
        self.SERIES_VERSION = 4

        #Define message fields
        self.NodeID = 0   #int64
        self.Coordinates = Location3D.Location3D()   #Location3D
        self.AssociatedEdges = []   #int64


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.NodeID))
        buffer.extend(struct.pack("B", self.Coordinates != None ))
        if self.Coordinates != None:
            buffer.extend(struct.pack(">q", self.Coordinates.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.Coordinates.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.Coordinates.SERIES_VERSION))
            buffer.extend(self.Coordinates.pack())
        buffer.extend(struct.pack(">H", len(self.AssociatedEdges) ))
        for x in self.AssociatedEdges:
            buffer.extend(struct.pack(">q", x ))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.NodeID = struct.unpack_from(">q", buffer, _pos)[0]
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
            self.Coordinates = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.Coordinates.unpack(buffer, _pos)
        else:
            self.Coordinates = None
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.AssociatedEdges = [None] * _arraylen
        if _arraylen > 0:
            self.AssociatedEdges = struct.unpack_from(">" + repr(_arraylen) + "q", buffer, _pos )
            _pos += 8 * _arraylen
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "NodeID" and len(e.childNodes) > 0 :
                    self.NodeID = int(e.childNodes[0].nodeValue)
                elif e.localName == "Coordinates" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.Coordinates = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.Coordinates != None:
                                self.Coordinates.unpackFromXMLNode(n, seriesFactory)
                elif e.localName == "AssociatedEdges" and len(e.childNodes) > 0 :
                    self.AssociatedEdges = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.AssociatedEdges.append( int(c.childNodes[0].nodeValue) )

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "NodeID":
                self.NodeID = d[key]
            elif key == "Coordinates":
                self.Coordinates = seriesFactory.unpackFromDict(d[key])
            elif key == "AssociatedEdges":
                self.AssociatedEdges = []
                for c in d[key]:
                    self.AssociatedEdges.append( c )

        return

    def get_NodeID(self):
        return self.NodeID

    def set_NodeID(self, value):
        self.NodeID = int( value )

    def get_Coordinates(self):
        return self.Coordinates

    def set_Coordinates(self, value):
        self.Coordinates = value 

    def get_AssociatedEdges(self):
        return self.AssociatedEdges



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From GraphNode:\n"
        buf +=    "NodeID = " + str( self.NodeID ) + "\n" 
        buf +=    "Coordinates = " + str( self.Coordinates ) + "\n" 
        buf +=    "AssociatedEdges = " + str( self.AssociatedEdges ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "ROUTE") or (len("ROUTE") == 0): # this should never happen
        	# Checks for "ROUTE" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/GraphNode")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("ROUTE" + "/GraphNode")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['NodeID'] = self.NodeID
        if self.Coordinates == None:
            d['Coordinates'] = None
        else:
            d['Coordinates'] = self.Coordinates.toDict()
        d['AssociatedEdges'] = []
        for x in self.AssociatedEdges:
            d['AssociatedEdges'].append(x)

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
        str = ws + '<GraphNode Series="ROUTE" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</GraphNode>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<NodeID>" + str(self.NodeID) + "</NodeID>\n"
        if self.Coordinates != None:
            buf += ws + "<Coordinates>\n"
            buf += ws + self.Coordinates.toXMLStr(ws + "    ") 
            buf += ws + "</Coordinates>\n"
        buf += ws + "<AssociatedEdges>\n"
        for x in self.AssociatedEdges:
            buf += ws + "<int64>" + str(x) + "</int64>\n"
        buf += ws + "</AssociatedEdges>\n"

        return buf
        
