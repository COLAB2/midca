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


class GraphEdge(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 2
        self.SERIES_NAME = "ROUTE"
        self.FULL_LMCP_TYPE_NAME = "uxas.messages.route.GraphEdge"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5931053054693474304
        self.SERIES_VERSION = 4

        #Define message fields
        self.EdgeID = 0   #int64
        self.StartNode = 0   #int64
        self.EndNode = 0   #int64
        self.Waypoints = []   #Location3D


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.EdgeID))
        buffer.extend(struct.pack(">q", self.StartNode))
        buffer.extend(struct.pack(">q", self.EndNode))
        buffer.extend(struct.pack(">H", len(self.Waypoints) ))
        for x in self.Waypoints:
           buffer.extend(struct.pack("B", x != None ))
           if x != None:
               buffer.extend(struct.pack(">q", x.SERIES_NAME_ID))
               buffer.extend(struct.pack(">I", x.LMCP_TYPE))
               buffer.extend(struct.pack(">H", x.SERIES_VERSION))
               buffer.extend(x.pack())

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.EdgeID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.StartNode = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.EndNode = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.Waypoints = [None] * _arraylen
        for x in range(_arraylen):
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
                self.Waypoints[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.Waypoints[x].unpack(buffer, _pos)
            else:
                self.Waypoints[x] = None
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "EdgeID" and len(e.childNodes) > 0 :
                    self.EdgeID = int(e.childNodes[0].nodeValue)
                elif e.localName == "StartNode" and len(e.childNodes) > 0 :
                    self.StartNode = int(e.childNodes[0].nodeValue)
                elif e.localName == "EndNode" and len(e.childNodes) > 0 :
                    self.EndNode = int(e.childNodes[0].nodeValue)
                elif e.localName == "Waypoints" and len(e.childNodes) > 0 :
                    self.Waypoints = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.Waypoints.append(obj)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "EdgeID":
                self.EdgeID = d[key]
            elif key == "StartNode":
                self.StartNode = d[key]
            elif key == "EndNode":
                self.EndNode = d[key]
            elif key == "Waypoints":
                self.Waypoints = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.Waypoints.append(obj)

        return

    def get_EdgeID(self):
        return self.EdgeID

    def set_EdgeID(self, value):
        self.EdgeID = int( value )

    def get_StartNode(self):
        return self.StartNode

    def set_StartNode(self, value):
        self.StartNode = int( value )

    def get_EndNode(self):
        return self.EndNode

    def set_EndNode(self, value):
        self.EndNode = int( value )

    def get_Waypoints(self):
        return self.Waypoints



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From GraphEdge:\n"
        buf +=    "EdgeID = " + str( self.EdgeID ) + "\n" 
        buf +=    "StartNode = " + str( self.StartNode ) + "\n" 
        buf +=    "EndNode = " + str( self.EndNode ) + "\n" 
        buf +=    "Waypoints = " + str( self.Waypoints ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "ROUTE") or (len("ROUTE") == 0): # this should never happen
        	# Checks for "ROUTE" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/GraphEdge")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("ROUTE" + "/GraphEdge")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['EdgeID'] = self.EdgeID
        d['StartNode'] = self.StartNode
        d['EndNode'] = self.EndNode
        d['Waypoints'] = []
        for x in self.Waypoints:
            if x == None:
                d['Waypoints'].append(None)
            else:
                d['Waypoints'].append(x.toDict())

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
        str = ws + '<GraphEdge Series="ROUTE" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</GraphEdge>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<EdgeID>" + str(self.EdgeID) + "</EdgeID>\n"
        buf += ws + "<StartNode>" + str(self.StartNode) + "</StartNode>\n"
        buf += ws + "<EndNode>" + str(self.EndNode) + "</EndNode>\n"
        buf += ws + "<Waypoints>\n"
        for x in self.Waypoints:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</Waypoints>\n"

        return buf
        
