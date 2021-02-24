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

from midca.domains.tsp.lmcp.py.uxas.messages.route import RoadPointsConstraints


class RoadPointsRequest(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 13
        self.SERIES_NAME = "ROUTE"
        self.FULL_LMCP_TYPE_NAME = "uxas.messages.route.RoadPointsRequest"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5931053054693474304
        self.SERIES_VERSION = 4

        #Define message fields
        self.RequestID = 0   #int64
        self.RoadPointsRequests = []   #RoadPointsConstraints


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.RequestID))
        buffer.extend(struct.pack(">H", len(self.RoadPointsRequests) ))
        for x in self.RoadPointsRequests:
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
        self.RequestID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.RoadPointsRequests = [None] * _arraylen
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
                self.RoadPointsRequests[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.RoadPointsRequests[x].unpack(buffer, _pos)
            else:
                self.RoadPointsRequests[x] = None
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "RequestID" and len(e.childNodes) > 0 :
                    self.RequestID = int(e.childNodes[0].nodeValue)
                elif e.localName == "RoadPointsRequests" and len(e.childNodes) > 0 :
                    self.RoadPointsRequests = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.RoadPointsRequests.append(obj)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "RequestID":
                self.RequestID = d[key]
            elif key == "RoadPointsRequests":
                self.RoadPointsRequests = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.RoadPointsRequests.append(obj)

        return

    def get_RequestID(self):
        return self.RequestID

    def set_RequestID(self, value):
        self.RequestID = int( value )

    def get_RoadPointsRequests(self):
        return self.RoadPointsRequests



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From RoadPointsRequest:\n"
        buf +=    "RequestID = " + str( self.RequestID ) + "\n" 
        buf +=    "RoadPointsRequests = " + str( self.RoadPointsRequests ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "ROUTE") or (len("ROUTE") == 0): # this should never happen
        	# Checks for "ROUTE" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/RoadPointsRequest")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("ROUTE" + "/RoadPointsRequest")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['RequestID'] = self.RequestID
        d['RoadPointsRequests'] = []
        for x in self.RoadPointsRequests:
            if x == None:
                d['RoadPointsRequests'].append(None)
            else:
                d['RoadPointsRequests'].append(x.toDict())

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
        str = ws + '<RoadPointsRequest Series="ROUTE" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</RoadPointsRequest>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<RequestID>" + str(self.RequestID) + "</RequestID>\n"
        buf += ws + "<RoadPointsRequests>\n"
        for x in self.RoadPointsRequests:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</RoadPointsRequests>\n"

        return buf
        
