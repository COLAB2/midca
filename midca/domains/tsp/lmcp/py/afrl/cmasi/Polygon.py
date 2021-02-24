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

from midca.domains.tsp.lmcp.py.afrl.cmasi import AbstractGeometry
from midca.domains.tsp.lmcp.py.afrl.cmasi import Location3D


class Polygon(AbstractGeometry.AbstractGeometry):

    def __init__(self):
        AbstractGeometry.AbstractGeometry.__init__(self)
        self.LMCP_TYPE = 42
        self.SERIES_NAME = "CMASI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.Polygon"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 4849604199710720000
        self.SERIES_VERSION = 3

        #Define message fields
        self.BoundaryPoints = []   #Location3D


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(AbstractGeometry.AbstractGeometry.pack(self))
        buffer.extend(struct.pack(">H", len(self.BoundaryPoints) ))
        for x in self.BoundaryPoints:
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
        _pos = AbstractGeometry.AbstractGeometry.unpack(self, buffer, _pos)
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.BoundaryPoints = [None] * _arraylen
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
                self.BoundaryPoints[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.BoundaryPoints[x].unpack(buffer, _pos)
            else:
                self.BoundaryPoints[x] = None
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        AbstractGeometry.AbstractGeometry.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "BoundaryPoints" and len(e.childNodes) > 0 :
                    self.BoundaryPoints = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.BoundaryPoints.append(obj)

        return

    def unpackFromDict(self, d, seriesFactory):
        AbstractGeometry.AbstractGeometry.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "BoundaryPoints":
                self.BoundaryPoints = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.BoundaryPoints.append(obj)

        return

    def get_BoundaryPoints(self):
        return self.BoundaryPoints



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = AbstractGeometry.AbstractGeometry.toString(self)
        buf += "From Polygon:\n"
        buf +=    "BoundaryPoints = " + str( self.BoundaryPoints ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "CMASI") or (len("CMASI") == 0): # this should never happen
        	# Checks for "CMASI" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/Polygon")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("CMASI" + "/Polygon")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        AbstractGeometry.AbstractGeometry.toDictMembers(self, d)
        d['BoundaryPoints'] = []
        for x in self.BoundaryPoints:
            if x == None:
                d['BoundaryPoints'].append(None)
            else:
                d['BoundaryPoints'].append(x.toDict())

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
        str = ws + '<Polygon Series="CMASI" >\n';
        #str +=AbstractGeometry.AbstractGeometry.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</Polygon>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += AbstractGeometry.AbstractGeometry.toXMLMembersStr(self, ws)
        buf += ws + "<BoundaryPoints>\n"
        for x in self.BoundaryPoints:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</BoundaryPoints>\n"

        return buf
        
