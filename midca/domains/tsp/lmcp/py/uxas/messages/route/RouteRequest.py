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

from midca.domains.tsp.lmcp.py.uxas.messages.route import RouteConstraints


class RouteRequest(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 5
        self.SERIES_NAME = "ROUTE"
        self.FULL_LMCP_TYPE_NAME = "uxas.messages.route.RouteRequest"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5931053054693474304
        self.SERIES_VERSION = 4

        #Define message fields
        self.RequestID = 0   #int64
        self.AssociatedTaskID = 0   #int64
        self.VehicleID = []   #int64
        self.OperatingRegion = 0   #int64
        self.RouteRequests = []   #RouteConstraints
        self.IsCostOnlyRequest = True   #bool


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.RequestID))
        buffer.extend(struct.pack(">q", self.AssociatedTaskID))
        buffer.extend(struct.pack(">H", len(self.VehicleID) ))
        for x in self.VehicleID:
            buffer.extend(struct.pack(">q", x ))
        buffer.extend(struct.pack(">q", self.OperatingRegion))
        buffer.extend(struct.pack(">H", len(self.RouteRequests) ))
        for x in self.RouteRequests:
           buffer.extend(struct.pack("B", x != None ))
           if x != None:
               buffer.extend(struct.pack(">q", x.SERIES_NAME_ID))
               buffer.extend(struct.pack(">I", x.LMCP_TYPE))
               buffer.extend(struct.pack(">H", x.SERIES_VERSION))
               buffer.extend(x.pack())
        boolChar = 1 if self.IsCostOnlyRequest == True else 0
        buffer.extend(struct.pack(">B",boolChar))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.RequestID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.AssociatedTaskID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.VehicleID = [None] * _arraylen
        if _arraylen > 0:
            self.VehicleID = struct.unpack_from(">" + repr(_arraylen) + "q", buffer, _pos )
            _pos += 8 * _arraylen
        self.OperatingRegion = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.RouteRequests = [None] * _arraylen
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
                self.RouteRequests[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.RouteRequests[x].unpack(buffer, _pos)
            else:
                self.RouteRequests[x] = None
        boolChar = struct.unpack_from(">B", buffer, _pos)[0]
        self.IsCostOnlyRequest = True if boolChar == 1 else False
        _pos += 1
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "RequestID" and len(e.childNodes) > 0 :
                    self.RequestID = int(e.childNodes[0].nodeValue)
                elif e.localName == "AssociatedTaskID" and len(e.childNodes) > 0 :
                    self.AssociatedTaskID = int(e.childNodes[0].nodeValue)
                elif e.localName == "VehicleID" and len(e.childNodes) > 0 :
                    self.VehicleID = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.VehicleID.append( int(c.childNodes[0].nodeValue) )
                elif e.localName == "OperatingRegion" and len(e.childNodes) > 0 :
                    self.OperatingRegion = int(e.childNodes[0].nodeValue)
                elif e.localName == "RouteRequests" and len(e.childNodes) > 0 :
                    self.RouteRequests = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.RouteRequests.append(obj)
                elif e.localName == "IsCostOnlyRequest" and len(e.childNodes) > 0 :
                    self.IsCostOnlyRequest = e.childNodes[0].nodeValue.lower() == 'true'

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "RequestID":
                self.RequestID = d[key]
            elif key == "AssociatedTaskID":
                self.AssociatedTaskID = d[key]
            elif key == "VehicleID":
                self.VehicleID = []
                for c in d[key]:
                    self.VehicleID.append( c )
            elif key == "OperatingRegion":
                self.OperatingRegion = d[key]
            elif key == "RouteRequests":
                self.RouteRequests = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.RouteRequests.append(obj)
            elif key == "IsCostOnlyRequest":
                self.IsCostOnlyRequest = d[key]

        return

    def get_RequestID(self):
        return self.RequestID

    def set_RequestID(self, value):
        self.RequestID = int( value )

    def get_AssociatedTaskID(self):
        return self.AssociatedTaskID

    def set_AssociatedTaskID(self, value):
        self.AssociatedTaskID = int( value )

    def get_VehicleID(self):
        return self.VehicleID

    def get_OperatingRegion(self):
        return self.OperatingRegion

    def set_OperatingRegion(self, value):
        self.OperatingRegion = int( value )

    def get_RouteRequests(self):
        return self.RouteRequests

    def get_IsCostOnlyRequest(self):
        return self.IsCostOnlyRequest

    def set_IsCostOnlyRequest(self, value):
        self.IsCostOnlyRequest = bool( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From RouteRequest:\n"
        buf +=    "RequestID = " + str( self.RequestID ) + "\n" 
        buf +=    "AssociatedTaskID = " + str( self.AssociatedTaskID ) + "\n" 
        buf +=    "VehicleID = " + str( self.VehicleID ) + "\n" 
        buf +=    "OperatingRegion = " + str( self.OperatingRegion ) + "\n" 
        buf +=    "RouteRequests = " + str( self.RouteRequests ) + "\n" 
        buf +=    "IsCostOnlyRequest = " + str( self.IsCostOnlyRequest ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "ROUTE") or (len("ROUTE") == 0): # this should never happen
        	# Checks for "ROUTE" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/RouteRequest")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("ROUTE" + "/RouteRequest")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['RequestID'] = self.RequestID
        d['AssociatedTaskID'] = self.AssociatedTaskID
        d['VehicleID'] = []
        for x in self.VehicleID:
            d['VehicleID'].append(x)
        d['OperatingRegion'] = self.OperatingRegion
        d['RouteRequests'] = []
        for x in self.RouteRequests:
            if x == None:
                d['RouteRequests'].append(None)
            else:
                d['RouteRequests'].append(x.toDict())
        d['IsCostOnlyRequest'] = self.IsCostOnlyRequest

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
        str = ws + '<RouteRequest Series="ROUTE" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</RouteRequest>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<RequestID>" + str(self.RequestID) + "</RequestID>\n"
        buf += ws + "<AssociatedTaskID>" + str(self.AssociatedTaskID) + "</AssociatedTaskID>\n"
        buf += ws + "<VehicleID>\n"
        for x in self.VehicleID:
            buf += ws + "<int64>" + str(x) + "</int64>\n"
        buf += ws + "</VehicleID>\n"
        buf += ws + "<OperatingRegion>" + str(self.OperatingRegion) + "</OperatingRegion>\n"
        buf += ws + "<RouteRequests>\n"
        for x in self.RouteRequests:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</RouteRequests>\n"
        buf += ws + "<IsCostOnlyRequest>" + ('True' if self.IsCostOnlyRequest else 'False') + "</IsCostOnlyRequest>\n"

        return buf
        
