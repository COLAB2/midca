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
from midca.domains.tsp.lmcp.py.afrl.impact import AreaSearchPattern


class PatternSearchTask(SearchTask.SearchTask):

    def __init__(self):
        SearchTask.SearchTask.__init__(self)
        self.LMCP_TYPE = 23
        self.SERIES_NAME = "IMPACT"
        self.FULL_LMCP_TYPE_NAME = "afrl.impact.PatternSearchTask"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5281966179208134656
        self.SERIES_VERSION = 14

        #Define message fields
        self.SearchLocationID = 0   #int64
        self.SearchLocation = None   #Location3D
        self.Pattern = AreaSearchPattern.AreaSearchPattern.Spiral   #AreaSearchPattern
        self.Extent = 0.0   #real32


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
        buffer.extend(struct.pack(">i", self.Pattern))
        buffer.extend(struct.pack(">f", self.Extent))

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
        self.Pattern = struct.unpack_from(">i", buffer, _pos)[0]
        _pos += 4
        self.Extent = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
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
                elif e.localName == "Pattern" and len(e.childNodes) > 0 :
                    self.Pattern = AreaSearchPattern.get_AreaSearchPattern_str(e.childNodes[0].nodeValue)
                elif e.localName == "Extent" and len(e.childNodes) > 0 :
                    self.Extent = float(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        SearchTask.SearchTask.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "SearchLocationID":
                self.SearchLocationID = d[key]
            elif key == "SearchLocation":
                self.SearchLocation = seriesFactory.unpackFromDict(d[key])
            elif key == "Pattern":
                self.Pattern = d[key]
            elif key == "Extent":
                self.Extent = d[key]

        return

    def get_SearchLocationID(self):
        return self.SearchLocationID

    def set_SearchLocationID(self, value):
        self.SearchLocationID = int( value )

    def get_SearchLocation(self):
        return self.SearchLocation

    def set_SearchLocation(self, value):
        self.SearchLocation = value 

    def get_Pattern(self):
        return self.Pattern

    def set_Pattern(self, value):
        self.Pattern = value 

    def get_Extent(self):
        return self.Extent

    def set_Extent(self, value):
        self.Extent = float( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = SearchTask.SearchTask.toString(self)
        buf += "From PatternSearchTask:\n"
        buf +=    "SearchLocationID = " + str( self.SearchLocationID ) + "\n" 
        buf +=    "SearchLocation = " + str( self.SearchLocation ) + "\n" 
        buf +=    "Pattern = " + str( self.Pattern ) + "\n" 
        buf +=    "Extent = " + str( self.Extent ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "IMPACT") or (len("IMPACT") == 0): # this should never happen
        	# Checks for "IMPACT" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/PatternSearchTask")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("IMPACT" + "/PatternSearchTask")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        SearchTask.SearchTask.toDictMembers(self, d)
        d['SearchLocationID'] = self.SearchLocationID
        if self.SearchLocation == None:
            d['SearchLocation'] = None
        else:
            d['SearchLocation'] = self.SearchLocation.toDict()
        d['Pattern'] = self.Pattern
        d['Extent'] = self.Extent

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
        str = ws + '<PatternSearchTask Series="IMPACT" >\n';
        #str +=SearchTask.SearchTask.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</PatternSearchTask>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += SearchTask.SearchTask.toXMLMembersStr(self, ws)
        buf += ws + "<SearchLocationID>" + str(self.SearchLocationID) + "</SearchLocationID>\n"
        if self.SearchLocation != None:
            buf += ws + "<SearchLocation>\n"
            buf += ws + self.SearchLocation.toXMLStr(ws + "    ") 
            buf += ws + "</SearchLocation>\n"
        buf += ws + "<Pattern>" + AreaSearchPattern.get_AreaSearchPattern_int(self.Pattern) + "</Pattern>\n"
        buf += ws + "<Extent>" + str(self.Extent) + "</Extent>\n"

        return buf
        
