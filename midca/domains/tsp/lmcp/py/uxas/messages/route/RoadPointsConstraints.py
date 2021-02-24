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


class RoadPointsConstraints(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 12
        self.SERIES_NAME = "ROUTE"
        self.FULL_LMCP_TYPE_NAME = "uxas.messages.route.RoadPointsConstraints"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5931053054693474304
        self.SERIES_VERSION = 4

        #Define message fields
        self.RoadPointsID = 0   #int64
        self.StartLocation = Location3D.Location3D()   #Location3D
        self.EndLocation = Location3D.Location3D()   #Location3D


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">q", self.RoadPointsID))
        buffer.extend(struct.pack("B", self.StartLocation != None ))
        if self.StartLocation != None:
            buffer.extend(struct.pack(">q", self.StartLocation.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.StartLocation.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.StartLocation.SERIES_VERSION))
            buffer.extend(self.StartLocation.pack())
        buffer.extend(struct.pack("B", self.EndLocation != None ))
        if self.EndLocation != None:
            buffer.extend(struct.pack(">q", self.EndLocation.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.EndLocation.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.EndLocation.SERIES_VERSION))
            buffer.extend(self.EndLocation.pack())

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.RoadPointsID = struct.unpack_from(">q", buffer, _pos)[0]
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
            self.EndLocation = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.EndLocation.unpack(buffer, _pos)
        else:
            self.EndLocation = None
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "RoadPointsID" and len(e.childNodes) > 0 :
                    self.RoadPointsID = int(e.childNodes[0].nodeValue)
                elif e.localName == "StartLocation" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.StartLocation = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.StartLocation != None:
                                self.StartLocation.unpackFromXMLNode(n, seriesFactory)
                elif e.localName == "EndLocation" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.EndLocation = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.EndLocation != None:
                                self.EndLocation.unpackFromXMLNode(n, seriesFactory)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "RoadPointsID":
                self.RoadPointsID = d[key]
            elif key == "StartLocation":
                self.StartLocation = seriesFactory.unpackFromDict(d[key])
            elif key == "EndLocation":
                self.EndLocation = seriesFactory.unpackFromDict(d[key])

        return

    def get_RoadPointsID(self):
        return self.RoadPointsID

    def set_RoadPointsID(self, value):
        self.RoadPointsID = int( value )

    def get_StartLocation(self):
        return self.StartLocation

    def set_StartLocation(self, value):
        self.StartLocation = value 

    def get_EndLocation(self):
        return self.EndLocation

    def set_EndLocation(self, value):
        self.EndLocation = value 



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From RoadPointsConstraints:\n"
        buf +=    "RoadPointsID = " + str( self.RoadPointsID ) + "\n" 
        buf +=    "StartLocation = " + str( self.StartLocation ) + "\n" 
        buf +=    "EndLocation = " + str( self.EndLocation ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "ROUTE") or (len("ROUTE") == 0): # this should never happen
        	# Checks for "ROUTE" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/RoadPointsConstraints")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("ROUTE" + "/RoadPointsConstraints")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['RoadPointsID'] = self.RoadPointsID
        if self.StartLocation == None:
            d['StartLocation'] = None
        else:
            d['StartLocation'] = self.StartLocation.toDict()
        if self.EndLocation == None:
            d['EndLocation'] = None
        else:
            d['EndLocation'] = self.EndLocation.toDict()

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
        str = ws + '<RoadPointsConstraints Series="ROUTE" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</RoadPointsConstraints>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<RoadPointsID>" + str(self.RoadPointsID) + "</RoadPointsID>\n"
        if self.StartLocation != None:
            buf += ws + "<StartLocation>\n"
            buf += ws + self.StartLocation.toXMLStr(ws + "    ") 
            buf += ws + "</StartLocation>\n"
        if self.EndLocation != None:
            buf += ws + "<EndLocation>\n"
            buf += ws + self.EndLocation.toXMLStr(ws + "    ") 
            buf += ws + "</EndLocation>\n"

        return buf
        
