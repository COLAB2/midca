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


class WaterReport(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 33
        self.SERIES_NAME = "IMPACT"
        self.FULL_LMCP_TYPE_NAME = "afrl.impact.WaterReport"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 5281966179208134656
        self.SERIES_VERSION = 14

        #Define message fields
        self.Area = AbstractGeometry.AbstractGeometry()   #AbstractGeometry
        self.CurrentSpeed = 0   #real32
        self.CurrentDirection = 0   #real32
        self.WaveDirection = 0   #real32
        self.WaveHeight = 0   #real32


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack("B", self.Area != None ))
        if self.Area != None:
            buffer.extend(struct.pack(">q", self.Area.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.Area.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.Area.SERIES_VERSION))
            buffer.extend(self.Area.pack())
        buffer.extend(struct.pack(">f", self.CurrentSpeed))
        buffer.extend(struct.pack(">f", self.CurrentDirection))
        buffer.extend(struct.pack(">f", self.WaveDirection))
        buffer.extend(struct.pack(">f", self.WaveHeight))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
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
            self.Area = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.Area.unpack(buffer, _pos)
        else:
            self.Area = None
        self.CurrentSpeed = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.CurrentDirection = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.WaveDirection = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.WaveHeight = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "Area" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.Area = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.Area != None:
                                self.Area.unpackFromXMLNode(n, seriesFactory)
                elif e.localName == "CurrentSpeed" and len(e.childNodes) > 0 :
                    self.CurrentSpeed = float(e.childNodes[0].nodeValue)
                elif e.localName == "CurrentDirection" and len(e.childNodes) > 0 :
                    self.CurrentDirection = float(e.childNodes[0].nodeValue)
                elif e.localName == "WaveDirection" and len(e.childNodes) > 0 :
                    self.WaveDirection = float(e.childNodes[0].nodeValue)
                elif e.localName == "WaveHeight" and len(e.childNodes) > 0 :
                    self.WaveHeight = float(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "Area":
                self.Area = seriesFactory.unpackFromDict(d[key])
            elif key == "CurrentSpeed":
                self.CurrentSpeed = d[key]
            elif key == "CurrentDirection":
                self.CurrentDirection = d[key]
            elif key == "WaveDirection":
                self.WaveDirection = d[key]
            elif key == "WaveHeight":
                self.WaveHeight = d[key]

        return

    def get_Area(self):
        return self.Area

    def set_Area(self, value):
        self.Area = value 

    def get_CurrentSpeed(self):
        return self.CurrentSpeed

    def set_CurrentSpeed(self, value):
        self.CurrentSpeed = float( value )

    def get_CurrentDirection(self):
        return self.CurrentDirection

    def set_CurrentDirection(self, value):
        self.CurrentDirection = float( value )

    def get_WaveDirection(self):
        return self.WaveDirection

    def set_WaveDirection(self, value):
        self.WaveDirection = float( value )

    def get_WaveHeight(self):
        return self.WaveHeight

    def set_WaveHeight(self, value):
        self.WaveHeight = float( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From WaterReport:\n"
        buf +=    "Area = " + str( self.Area ) + "\n" 
        buf +=    "CurrentSpeed = " + str( self.CurrentSpeed ) + "\n" 
        buf +=    "CurrentDirection = " + str( self.CurrentDirection ) + "\n" 
        buf +=    "WaveDirection = " + str( self.WaveDirection ) + "\n" 
        buf +=    "WaveHeight = " + str( self.WaveHeight ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "IMPACT") or (len("IMPACT") == 0): # this should never happen
        	# Checks for "IMPACT" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/WaterReport")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("IMPACT" + "/WaterReport")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        if self.Area == None:
            d['Area'] = None
        else:
            d['Area'] = self.Area.toDict()
        d['CurrentSpeed'] = self.CurrentSpeed
        d['CurrentDirection'] = self.CurrentDirection
        d['WaveDirection'] = self.WaveDirection
        d['WaveHeight'] = self.WaveHeight

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
        str = ws + '<WaterReport Series="IMPACT" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</WaterReport>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        if self.Area != None:
            buf += ws + "<Area>\n"
            buf += ws + self.Area.toXMLStr(ws + "    ") 
            buf += ws + "</Area>\n"
        buf += ws + "<CurrentSpeed>" + str(self.CurrentSpeed) + "</CurrentSpeed>\n"
        buf += ws + "<CurrentDirection>" + str(self.CurrentDirection) + "</CurrentDirection>\n"
        buf += ws + "<WaveDirection>" + str(self.WaveDirection) + "</WaveDirection>\n"
        buf += ws + "<WaveHeight>" + str(self.WaveHeight) + "</WaveHeight>\n"

        return buf
        
