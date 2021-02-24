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
from midca.domains.tsp.lmcp.py.afrl.cmasi import Wedge
from midca.domains.tsp.lmcp.py.afrl.cmasi import LoiterAction


class ImpactPointSearchTask(SearchTask.SearchTask):

    def __init__(self):
        SearchTask.SearchTask.__init__(self)
        self.LMCP_TYPE = 22
        self.SERIES_NAME = "IMPACT"
        self.FULL_LMCP_TYPE_NAME = "afrl.impact.ImpactPointSearchTask"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5281966179208134656
        self.SERIES_VERSION = 14

        #Define message fields
        self.SearchLocationID = 0   #int64
        self.SearchLocation = None   #Location3D
        self.StandoffDistance = 0   #real32
        self.ViewAngleList = []   #Wedge
        self.DesiredAction = None   #LoiterAction


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(SearchTask.SearchTask.pack(self))
        buffer.extend(struct.pack(">q", self.SearchLocationID))
        buffer.extend(struct.pack("B", self.SearchLocation != None ))
        if self.SearchLocation != None:
            buffer.extend(struct.pack(">q", self.SearchLocation.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.SearchLocation.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.SearchLocation.SERIES_VERSION))
            buffer.extend(self.SearchLocation.pack())
        buffer.extend(struct.pack(">f", self.StandoffDistance))
        buffer.extend(struct.pack(">H", len(self.ViewAngleList) ))
        for x in self.ViewAngleList:
           buffer.extend(struct.pack("B", x != None ))
           if x != None:
               buffer.extend(struct.pack(">q", x.SERIES_NAME_ID))
               buffer.extend(struct.pack(">I", x.LMCP_TYPE))
               buffer.extend(struct.pack(">H", x.SERIES_VERSION))
               buffer.extend(x.pack())
        buffer.extend(struct.pack("B", self.DesiredAction != None ))
        if self.DesiredAction != None:
            buffer.extend(struct.pack(">q", self.DesiredAction.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.DesiredAction.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.DesiredAction.SERIES_VERSION))
            buffer.extend(self.DesiredAction.pack())

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = SearchTask.SearchTask.unpack(self, buffer, _pos)
        self.SearchLocationID = struct.unpack_from(">q", buffer, _pos)[0]
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
            self.SearchLocation = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.SearchLocation.unpack(buffer, _pos)
        else:
            self.SearchLocation = None
        self.StandoffDistance = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
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
            self.DesiredAction = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.DesiredAction.unpack(buffer, _pos)
        else:
            self.DesiredAction = None
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        SearchTask.SearchTask.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "SearchLocationID" and len(e.childNodes) > 0 :
                    self.SearchLocationID = int(e.childNodes[0].nodeValue)
                elif e.localName == "SearchLocation" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.SearchLocation = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.SearchLocation != None:
                                self.SearchLocation.unpackFromXMLNode(n, seriesFactory)
                elif e.localName == "StandoffDistance" and len(e.childNodes) > 0 :
                    self.StandoffDistance = float(e.childNodes[0].nodeValue)
                elif e.localName == "ViewAngleList" and len(e.childNodes) > 0 :
                    self.ViewAngleList = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            obj = seriesFactory.createObjectByName(c.getAttribute('Series'), c.localName)
                            if obj != None:
                                obj.unpackFromXMLNode(c, seriesFactory)
                                self.ViewAngleList.append(obj)
                elif e.localName == "DesiredAction" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.DesiredAction = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.DesiredAction != None:
                                self.DesiredAction.unpackFromXMLNode(n, seriesFactory)

        return

    def unpackFromDict(self, d, seriesFactory):
        SearchTask.SearchTask.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "SearchLocationID":
                self.SearchLocationID = d[key]
            elif key == "SearchLocation":
                self.SearchLocation = seriesFactory.unpackFromDict(d[key])
            elif key == "StandoffDistance":
                self.StandoffDistance = d[key]
            elif key == "ViewAngleList":
                self.ViewAngleList = []
                for c in d[key]:
                    obj = seriesFactory.unpackFromDict(c)
                    if obj != None:
                        self.ViewAngleList.append(obj)
            elif key == "DesiredAction":
                self.DesiredAction = seriesFactory.unpackFromDict(d[key])

        return

    def get_SearchLocationID(self):
        return self.SearchLocationID

    def set_SearchLocationID(self, value):
        self.SearchLocationID = int( value )

    def get_SearchLocation(self):
        return self.SearchLocation

    def set_SearchLocation(self, value):
        self.SearchLocation = value 

    def get_StandoffDistance(self):
        return self.StandoffDistance

    def set_StandoffDistance(self, value):
        self.StandoffDistance = float( value )

    def get_ViewAngleList(self):
        return self.ViewAngleList

    def get_DesiredAction(self):
        return self.DesiredAction

    def set_DesiredAction(self, value):
        self.DesiredAction = value 



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = SearchTask.SearchTask.toString(self)
        buf += "From ImpactPointSearchTask:\n"
        buf +=    "SearchLocationID = " + str( self.SearchLocationID ) + "\n" 
        buf +=    "SearchLocation = " + str( self.SearchLocation ) + "\n" 
        buf +=    "StandoffDistance = " + str( self.StandoffDistance ) + "\n" 
        buf +=    "ViewAngleList = " + str( self.ViewAngleList ) + "\n" 
        buf +=    "DesiredAction = " + str( self.DesiredAction ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "IMPACT") or (len("IMPACT") == 0): # this should never happen
        	# Checks for "IMPACT" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/ImpactPointSearchTask")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("IMPACT" + "/ImpactPointSearchTask")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        SearchTask.SearchTask.toDictMembers(self, d)
        d['SearchLocationID'] = self.SearchLocationID
        if self.SearchLocation == None:
            d['SearchLocation'] = None
        else:
            d['SearchLocation'] = self.SearchLocation.toDict()
        d['StandoffDistance'] = self.StandoffDistance
        d['ViewAngleList'] = []
        for x in self.ViewAngleList:
            if x == None:
                d['ViewAngleList'].append(None)
            else:
                d['ViewAngleList'].append(x.toDict())
        if self.DesiredAction == None:
            d['DesiredAction'] = None
        else:
            d['DesiredAction'] = self.DesiredAction.toDict()

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
        str = ws + '<ImpactPointSearchTask Series="IMPACT" >\n';
        #str +=SearchTask.SearchTask.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</ImpactPointSearchTask>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += SearchTask.SearchTask.toXMLMembersStr(self, ws)
        buf += ws + "<SearchLocationID>" + str(self.SearchLocationID) + "</SearchLocationID>\n"
        if self.SearchLocation != None:
            buf += ws + "<SearchLocation>\n"
            buf += ws + self.SearchLocation.toXMLStr(ws + "    ") 
            buf += ws + "</SearchLocation>\n"
        buf += ws + "<StandoffDistance>" + str(self.StandoffDistance) + "</StandoffDistance>\n"
        buf += ws + "<ViewAngleList>\n"
        for x in self.ViewAngleList:
            if x == None:
                buf += ws + "    <null/>\n"
            else:
                buf += x.toXMLStr(ws + "    ") 
        buf += ws + "</ViewAngleList>\n"
        if self.DesiredAction != None:
            buf += ws + "<DesiredAction>\n"
            buf += ws + self.DesiredAction.toXMLStr(ws + "    ") 
            buf += ws + "</DesiredAction>\n"

        return buf
        
