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
from midca.domains.tsp.lmcp.py.afrl.cmasi import Location3D


class AngledAreaSearchTask(SearchTask.SearchTask):

    def __init__(self):
        SearchTask.SearchTask.__init__(self)
        self.LMCP_TYPE = 24
        self.SERIES_NAME = "IMPACT"
        self.FULL_LMCP_TYPE_NAME = "afrl.impact.AngledAreaSearchTask"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5281966179208134656
        self.SERIES_VERSION = 14

        #Define message fields
        self.SearchAreaID = 0   #int64
        self.SweepAngle = 0   #real32
        self.StartPoint = None   #Location3D


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(SearchTask.SearchTask.pack(self))
        buffer.extend(struct.pack(">q", self.SearchAreaID))
        buffer.extend(struct.pack(">f", self.SweepAngle))
        buffer.extend(struct.pack("B", self.StartPoint != None ))
        if self.StartPoint != None:
            buffer.extend(struct.pack(">q", self.StartPoint.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.StartPoint.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.StartPoint.SERIES_VERSION))
            buffer.extend(self.StartPoint.pack())

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = SearchTask.SearchTask.unpack(self, buffer, _pos)
        self.SearchAreaID = struct.unpack_from(">q", buffer, _pos)[0]
        _pos += 8
        self.SweepAngle = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
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
            self.StartPoint = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.StartPoint.unpack(buffer, _pos)
        else:
            self.StartPoint = None
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        SearchTask.SearchTask.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "SearchAreaID" and len(e.childNodes) > 0 :
                    self.SearchAreaID = int(e.childNodes[0].nodeValue)
                elif e.localName == "SweepAngle" and len(e.childNodes) > 0 :
                    self.SweepAngle = float(e.childNodes[0].nodeValue)
                elif e.localName == "StartPoint" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.StartPoint = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.StartPoint != None:
                                self.StartPoint.unpackFromXMLNode(n, seriesFactory)

        return

    def unpackFromDict(self, d, seriesFactory):
        SearchTask.SearchTask.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "SearchAreaID":
                self.SearchAreaID = d[key]
            elif key == "SweepAngle":
                self.SweepAngle = d[key]
            elif key == "StartPoint":
                self.StartPoint = seriesFactory.unpackFromDict(d[key])

        return

    def get_SearchAreaID(self):
        return self.SearchAreaID

    def set_SearchAreaID(self, value):
        self.SearchAreaID = int( value )

    def get_SweepAngle(self):
        return self.SweepAngle

    def set_SweepAngle(self, value):
        self.SweepAngle = float( value )

    def get_StartPoint(self):
        return self.StartPoint

    def set_StartPoint(self, value):
        self.StartPoint = value 



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = SearchTask.SearchTask.toString(self)
        buf += "From AngledAreaSearchTask:\n"
        buf +=    "SearchAreaID = " + str( self.SearchAreaID ) + "\n" 
        buf +=    "SweepAngle = " + str( self.SweepAngle ) + "\n" 
        buf +=    "StartPoint = " + str( self.StartPoint ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "IMPACT") or (len("IMPACT") == 0): # this should never happen
        	# Checks for "IMPACT" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/AngledAreaSearchTask")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("IMPACT" + "/AngledAreaSearchTask")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        SearchTask.SearchTask.toDictMembers(self, d)
        d['SearchAreaID'] = self.SearchAreaID
        d['SweepAngle'] = self.SweepAngle
        if self.StartPoint == None:
            d['StartPoint'] = None
        else:
            d['StartPoint'] = self.StartPoint.toDict()

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
        str = ws + '<AngledAreaSearchTask Series="IMPACT" >\n';
        #str +=SearchTask.SearchTask.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</AngledAreaSearchTask>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += SearchTask.SearchTask.toXMLMembersStr(self, ws)
        buf += ws + "<SearchAreaID>" + str(self.SearchAreaID) + "</SearchAreaID>\n"
        buf += ws + "<SweepAngle>" + str(self.SweepAngle) + "</SweepAngle>\n"
        if self.StartPoint != None:
            buf += ws + "<StartPoint>\n"
            buf += ws + self.StartPoint.toXMLStr(ws + "    ") 
            buf += ws + "</StartPoint>\n"

        return buf
        
