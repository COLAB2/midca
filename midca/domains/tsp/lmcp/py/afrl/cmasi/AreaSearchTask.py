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

from midca.domains.tsp.lmcp.py.afrl.cmasi import SearchTask
from midca.domains.tsp.lmcp.py.afrl.cmasi import AbstractGeometry
from midca.domains.tsp.lmcp.py.afrl.cmasi import Wedge


class AreaSearchTask(SearchTask.SearchTask):

    def __init__(self):
        SearchTask.SearchTask.__init__(self)
        self.LMCP_TYPE = 17
        self.SERIES_NAME = "CMASI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.AreaSearchTask"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 4849604199710720000
        self.SERIES_VERSION = 3

        #Define message fields
        self.SearchArea = AbstractGeometry.AbstractGeometry()   #AbstractGeometry
        self.ViewAngleList = []   #Wedge


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(SearchTask.SearchTask.pack(self))
        buffer.extend(struct.pack("B", self.SearchArea != None ))
        if self.SearchArea != None:
            buffer.extend(struct.pack(">q", self.SearchArea.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.SearchArea.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.SearchArea.SERIES_VERSION))
            buffer.extend(self.SearchArea.pack())
        buffer.extend(struct.pack(">H", len(self.ViewAngleList) ))
        for x in self.ViewAngleList:
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
        _pos = SearchTask.SearchTask.unpack(self, buffer, _pos)
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
            self.SearchArea = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.SearchArea.unpack(buffer, _pos)
        else:
            self.SearchArea = None
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.ViewAngleList = [None] * _arraylen
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
                self.ViewAngleList[x] = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
                _pos = self.ViewAngleList[x].unpack(buffer, _pos)
            else:
                self.ViewAngleList[x] = None
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        SearchTask.SearchTask.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "SearchArea" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.SearchArea = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.SearchArea != None:
                                self.SearchArea.unpackFromXMLNode(n, seriesFactory)
                elif e.localName == "ViewAngleList" and len(e.childNodes) > 0 :
                    self.ViewAngleList = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.ViewAngleList.append(obj)

        return

    def unpackFromDict(self, d, seriesFactory):
        SearchTask.SearchTask.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "SearchArea":
                self.SearchArea = seriesFactory.unpackFromDict(d[key])
            elif key == "ViewAngleList":
                self.ViewAngleList = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.ViewAngleList.append(obj)

        return

    def get_SearchArea(self):
        return self.SearchArea

    def set_SearchArea(self, value):
        self.SearchArea = value 

    def get_ViewAngleList(self):
        return self.ViewAngleList



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = SearchTask.SearchTask.toString(self)
        buf += "From AreaSearchTask:\n"
        buf +=    "SearchArea = " + str( self.SearchArea ) + "\n" 
        buf +=    "ViewAngleList = " + str( self.ViewAngleList ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "CMASI") or (len("CMASI") == 0): # this should never happen
        	# Checks for "CMASI" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/AreaSearchTask")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("CMASI" + "/AreaSearchTask")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        SearchTask.SearchTask.toDictMembers(self, d)
        if self.SearchArea == None:
            d['SearchArea'] = None
        else:
            d['SearchArea'] = self.SearchArea.toDict()
        d['ViewAngleList'] = []
        for x in self.ViewAngleList:
            if x == None:
                d['ViewAngleList'].append(None)
            else:
                d['ViewAngleList'].append(x.toDict())

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
        str = ws + '<AreaSearchTask Series="CMASI" >\n';
        #str +=SearchTask.SearchTask.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</AreaSearchTask>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += SearchTask.SearchTask.toXMLMembersStr(self, ws)
        if self.SearchArea != None:
            buf += ws + "<SearchArea>\n"
            buf += ws + self.SearchArea.toXMLStr(ws + "    ") 
            buf += ws + "</SearchArea>\n"
        buf += ws + "<ViewAngleList>\n"
        for x in self.ViewAngleList:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</ViewAngleList>\n"

        return buf
        
