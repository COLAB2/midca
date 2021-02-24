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

from midca.domains.tsp.lmcp.py.afrl.cmasi import Waypoint
from midca.domains.tsp.lmcp.py.afrl.cmasi import KeyValuePair


class RoutePlan(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 7
        self.SERIES_NAME = "ROUTE"
        self.FULL_LMCP_TYPE_NAME = "uxas.messages.route.RoutePlan"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5931053054693474304
        self.SERIES_VERSION = 4

        #Define message fields
        self.RouteID = 0   #int64
        self.Waypoints = []   #Waypoint
        self.RouteCost = -1   #int64
        self.RouteError = []   #KeyValuePair


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.RouteID))
        buffer.extend(struct.pack(">H", len(self.Waypoints) ))
        for x in self.Waypoints:
           buffer.extend(struct.pack("B", x != None ))
           if x != None:
               buffer.extend(struct.pack(">q", x.SERIES_NAME_ID))
               buffer.extend(struct.pack(">I", x.LMCP_TYPE))
               buffer.extend(struct.pack(">H", x.SERIES_VERSION))
               buffer.extend(x.pack())
        buffer.extend(struct.pack(">q", self.RouteCost))
        buffer.extend(struct.pack(">H", len(self.RouteError) ))
        for x in self.RouteError:
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
        self.RouteID = struct.unpack_from(">q", buffer, _pos)[0]
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
        self.RouteCost = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.RouteError = [None] * _arraylen
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
                self.RouteError[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.RouteError[x].unpack(buffer, _pos)
            else:
                self.RouteError[x] = None
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "RouteID" and len(e.childNodes) > 0 :
                    self.RouteID = int(e.childNodes[0].nodeValue)
                elif e.localName == "Waypoints" and len(e.childNodes) > 0 :
                    self.Waypoints = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.Waypoints.append(obj)
                elif e.localName == "RouteCost" and len(e.childNodes) > 0 :
                    self.RouteCost = int(e.childNodes[0].nodeValue)
                elif e.localName == "RouteError" and len(e.childNodes) > 0 :
                    self.RouteError = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.RouteError.append(obj)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "RouteID":
                self.RouteID = d[key]
            elif key == "Waypoints":
                self.Waypoints = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.Waypoints.append(obj)
            elif key == "RouteCost":
                self.RouteCost = d[key]
            elif key == "RouteError":
                self.RouteError = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.RouteError.append(obj)

        return

    def get_RouteID(self):
        return self.RouteID

    def set_RouteID(self, value):
        self.RouteID = int( value )

    def get_Waypoints(self):
        return self.Waypoints

    def get_RouteCost(self):
        return self.RouteCost

    def set_RouteCost(self, value):
        self.RouteCost = int( value )

    def get_RouteError(self):
        return self.RouteError



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From RoutePlan:\n"
        buf +=    "RouteID = " + str( self.RouteID ) + "\n" 
        buf +=    "Waypoints = " + str( self.Waypoints ) + "\n" 
        buf +=    "RouteCost = " + str( self.RouteCost ) + "\n" 
        buf +=    "RouteError = " + str( self.RouteError ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "ROUTE") or (len("ROUTE") == 0): # this should never happen
        	# Checks for "ROUTE" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/RoutePlan")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("ROUTE" + "/RoutePlan")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['RouteID'] = self.RouteID
        d['Waypoints'] = []
        for x in self.Waypoints:
            if x == None:
                d['Waypoints'].append(None)
            else:
                d['Waypoints'].append(x.toDict())
        d['RouteCost'] = self.RouteCost
        d['RouteError'] = []
        for x in self.RouteError:
            if x == None:
                d['RouteError'].append(None)
            else:
                d['RouteError'].append(x.toDict())

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
        str = ws + '<RoutePlan Series="ROUTE" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</RoutePlan>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<RouteID>" + str(self.RouteID) + "</RouteID>\n"
        buf += ws + "<Waypoints>\n"
        for x in self.Waypoints:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</Waypoints>\n"
        buf += ws + "<RouteCost>" + str(self.RouteCost) + "</RouteCost>\n"
        buf += ws + "<RouteError>\n"
        for x in self.RouteError:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</RouteError>\n"

        return buf
        
