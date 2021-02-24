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

from midca.domains.tsp.lmcp.py.afrl.cmasi import AbstractZone


class WeatherReport(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 55
        self.SERIES_NAME = "CMASI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.WeatherReport"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 4849604199710720000
        self.SERIES_VERSION = 3

        #Define message fields
        self.Area = None   #AbstractZone
        self.WindSpeed = 0   #real32
        self.WindDirection = 0   #real32
        self.Visibility = 0   #real32
        self.CloudCeiling = 0   #real32
        self.CloudCoverage = 0   #real32


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
        buffer.extend(struct.pack(">f", self.WindSpeed))
        buffer.extend(struct.pack(">f", self.WindDirection))
        buffer.extend(struct.pack(">f", self.Visibility))
        buffer.extend(struct.pack(">f", self.CloudCeiling))
        buffer.extend(struct.pack(">f", self.CloudCoverage))

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
        self.WindSpeed = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.WindDirection = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.Visibility = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.CloudCeiling = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.CloudCoverage = struct.unpack_from(">f", buffer, _pos)[0]
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
                elif e.localName == "WindSpeed" and len(e.childNodes) > 0 :
                    self.WindSpeed = float(e.childNodes[0].nodeValue)
                elif e.localName == "WindDirection" and len(e.childNodes) > 0 :
                    self.WindDirection = float(e.childNodes[0].nodeValue)
                elif e.localName == "Visibility" and len(e.childNodes) > 0 :
                    self.Visibility = float(e.childNodes[0].nodeValue)
                elif e.localName == "CloudCeiling" and len(e.childNodes) > 0 :
                    self.CloudCeiling = float(e.childNodes[0].nodeValue)
                elif e.localName == "CloudCoverage" and len(e.childNodes) > 0 :
                    self.CloudCoverage = float(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "Area":
                self.Area = seriesFactory.unpackFromDict(d[key])
            elif key == "WindSpeed":
                self.WindSpeed = d[key]
            elif key == "WindDirection":
                self.WindDirection = d[key]
            elif key == "Visibility":
                self.Visibility = d[key]
            elif key == "CloudCeiling":
                self.CloudCeiling = d[key]
            elif key == "CloudCoverage":
                self.CloudCoverage = d[key]

        return

    def get_Area(self):
        return self.Area

    def set_Area(self, value):
        self.Area = value 

    def get_WindSpeed(self):
        return self.WindSpeed

    def set_WindSpeed(self, value):
        self.WindSpeed = float( value )

    def get_WindDirection(self):
        return self.WindDirection

    def set_WindDirection(self, value):
        self.WindDirection = float( value )

    def get_Visibility(self):
        return self.Visibility

    def set_Visibility(self, value):
        self.Visibility = float( value )

    def get_CloudCeiling(self):
        return self.CloudCeiling

    def set_CloudCeiling(self, value):
        self.CloudCeiling = float( value )

    def get_CloudCoverage(self):
        return self.CloudCoverage

    def set_CloudCoverage(self, value):
        self.CloudCoverage = float( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From WeatherReport:\n"
        buf +=    "Area = " + str( self.Area ) + "\n" 
        buf +=    "WindSpeed = " + str( self.WindSpeed ) + "\n" 
        buf +=    "WindDirection = " + str( self.WindDirection ) + "\n" 
        buf +=    "Visibility = " + str( self.Visibility ) + "\n" 
        buf +=    "CloudCeiling = " + str( self.CloudCeiling ) + "\n" 
        buf +=    "CloudCoverage = " + str( self.CloudCoverage ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if (not "CMASI") or (len("CMASI") == 0): # this should never happen
        	# Checks for "CMASI" being None or empty
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/WeatherReport")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("CMASI" + "/WeatherReport")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        if self.Area == None:
            d['Area'] = None
        else:
            d['Area'] = self.Area.toDict()
        d['WindSpeed'] = self.WindSpeed
        d['WindDirection'] = self.WindDirection
        d['Visibility'] = self.Visibility
        d['CloudCeiling'] = self.CloudCeiling
        d['CloudCoverage'] = self.CloudCoverage

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
        str = ws + '<WeatherReport Series="CMASI" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</WeatherReport>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        if self.Area != None:
            buf += ws + "<Area>\n"
            buf += ws + self.Area.toXMLStr(ws + "    ") 
            buf += ws + "</Area>\n"
        buf += ws + "<WindSpeed>" + str(self.WindSpeed) + "</WindSpeed>\n"
        buf += ws + "<WindDirection>" + str(self.WindDirection) + "</WindDirection>\n"
        buf += ws + "<Visibility>" + str(self.Visibility) + "</Visibility>\n"
        buf += ws + "<CloudCeiling>" + str(self.CloudCeiling) + "</CloudCeiling>\n"
        buf += ws + "<CloudCoverage>" + str(self.CloudCoverage) + "</CloudCoverage>\n"

        return buf
        
