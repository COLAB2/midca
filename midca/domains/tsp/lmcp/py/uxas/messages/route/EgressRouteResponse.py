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


class EgressRouteResponse(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 11
        self.SERIES_NAME = "ROUTE"
        self.FULL_LMCP_TYPE_NAME = "uxas.messages.route.EgressRouteResponse"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5931053054693474304
        self.SERIES_VERSION = 4

        #Define message fields
        self.ResponseID = 0   #int64
        self.NodeLocations = []   #Location3D
        self.Headings = []   #real32


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.ResponseID))
        buffer.extend(struct.pack(">H", len(self.NodeLocations) ))
        for x in self.NodeLocations:
           buffer.extend(struct.pack("B", x != None ))
           if x != None:
               buffer.extend(struct.pack(">q", x.SERIES_NAME_ID))
               buffer.extend(struct.pack(">I", x.LMCP_TYPE))
               buffer.extend(struct.pack(">H", x.SERIES_VERSION))
               buffer.extend(x.pack())
        buffer.extend(struct.pack(">H", len(self.Headings) ))
        for x in self.Headings:
            buffer.extend(struct.pack(">f", x ))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.ResponseID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.NodeLocations = [None] * _arraylen
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
                self.NodeLocations[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.NodeLocations[x].unpack(buffer, _pos)
            else:
                self.NodeLocations[x] = None
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.Headings = [None] * _arraylen
        if _arraylen > 0:
            self.Headings = struct.unpack_from(">" + repr(_arraylen) + "f", buffer, _pos )
            _pos += 4 * _arraylen
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "ResponseID" and len(e.childNodes) > 0 :
                    self.ResponseID = int(e.childNodes[0].nodeValue)
                elif e.localName == "NodeLocations" and len(e.childNodes) > 0 :
                    self.NodeLocations = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.NodeLocations.append(obj)
                elif e.localName == "Headings" and len(e.childNodes) > 0 :
                    self.Headings = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.Headings.append( float(c.childNodes[0].nodeValue) )

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "ResponseID":
                self.ResponseID = d[key]
            elif key == "NodeLocations":
                self.NodeLocations = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.NodeLocations.append(obj)
            elif key == "Headings":
                self.Headings = []
                for c in d[key]:
                    self.Headings.append( c )

        return

    def get_ResponseID(self):
        return self.ResponseID

    def set_ResponseID(self, value):
        self.ResponseID = int( value )

    def get_NodeLocations(self):
        return self.NodeLocations

    def get_Headings(self):
        return self.Headings



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From EgressRouteResponse:\n"
        buf +=    "ResponseID = " + str( self.ResponseID ) + "\n" 
        buf +=    "NodeLocations = " + str( self.NodeLocations ) + "\n" 
        buf +=    "Headings = " + str( self.Headings ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "ROUTE") or (len("ROUTE") == 0): # this should never happen
        	# Checks for "ROUTE" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/EgressRouteResponse")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("ROUTE" + "/EgressRouteResponse")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['ResponseID'] = self.ResponseID
        d['NodeLocations'] = []
        for x in self.NodeLocations:
            if x == None:
                d['NodeLocations'].append(None)
            else:
                d['NodeLocations'].append(x.toDict())
        d['Headings'] = []
        for x in self.Headings:
            d['Headings'].append(x)

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
        str = ws + '<EgressRouteResponse Series="ROUTE" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</EgressRouteResponse>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<ResponseID>" + str(self.ResponseID) + "</ResponseID>\n"
        buf += ws + "<NodeLocations>\n"
        for x in self.NodeLocations:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</NodeLocations>\n"
        buf += ws + "<Headings>\n"
        for x in self.Headings:
            buf += ws + "<real32>" + str(x) + "</real32>\n"
        buf += ws + "</Headings>\n"

        return buf
        
