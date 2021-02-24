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


class EgressRouteRequest(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 10
        self.SERIES_NAME = "ROUTE"
        self.FULL_LMCP_TYPE_NAME = "uxas.messages.route.EgressRouteRequest"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5931053054693474304
        self.SERIES_VERSION = 4

        #Define message fields
        self.RequestID = 0   #int64
        self.StartLocation = Location3D.Location3D()   #Location3D
        self.Radius = 60   #real32


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.RequestID))
        buffer.extend(struct.pack("B", self.StartLocation != None ))
        if self.StartLocation != None:
            buffer.extend(struct.pack(">q", self.StartLocation.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.StartLocation.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.StartLocation.SERIES_VERSION))
            buffer.extend(self.StartLocation.pack())
        buffer.extend(struct.pack(">f", self.Radius))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.RequestID = struct.unpack_from(">q", buffer, _pos)[0]
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
            self.StartLocation = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.StartLocation.unpack(buffer, _pos)
        else:
            self.StartLocation = None
        self.Radius = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "RequestID" and len(e.childNodes) > 0 :
                    self.RequestID = int(e.childNodes[0].nodeValue)
                elif e.localName == "StartLocation" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.StartLocation = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.StartLocation != None:
                                self.StartLocation.unpackFromXMLNode(n, seriesFactory)
                elif e.localName == "Radius" and len(e.childNodes) > 0 :
                    self.Radius = float(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "RequestID":
                self.RequestID = d[key]
            elif key == "StartLocation":
                self.StartLocation = seriesFactory.unpackFromDict(d[key])
            elif key == "Radius":
                self.Radius = d[key]

        return

    def get_RequestID(self):
        return self.RequestID

    def set_RequestID(self, value):
        self.RequestID = int( value )

    def get_StartLocation(self):
        return self.StartLocation

    def set_StartLocation(self, value):
        self.StartLocation = value 

    def get_Radius(self):
        return self.Radius

    def set_Radius(self, value):
        self.Radius = float( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From EgressRouteRequest:\n"
        buf +=    "RequestID = " + str( self.RequestID ) + "\n" 
        buf +=    "StartLocation = " + str( self.StartLocation ) + "\n" 
        buf +=    "Radius = " + str( self.Radius ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "ROUTE") or (len("ROUTE") == 0): # this should never happen
        	# Checks for "ROUTE" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/EgressRouteRequest")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("ROUTE" + "/EgressRouteRequest")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['RequestID'] = self.RequestID
        if self.StartLocation == None:
            d['StartLocation'] = None
        else:
            d['StartLocation'] = self.StartLocation.toDict()
        d['Radius'] = self.Radius

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
        str = ws + '<EgressRouteRequest Series="ROUTE" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</EgressRouteRequest>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<RequestID>" + str(self.RequestID) + "</RequestID>\n"
        if self.StartLocation != None:
            buf += ws + "<StartLocation>\n"
            buf += ws + self.StartLocation.toXMLStr(ws + "    ") 
            buf += ws + "</StartLocation>\n"
        buf += ws + "<Radius>" + str(self.Radius) + "</Radius>\n"

        return buf
        
